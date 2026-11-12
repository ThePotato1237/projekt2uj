#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/flash.h"
#include "pico/cyw43_arch.h"
#include "hardware/adc.h"
#include "hardware/sync.h"
#include "btstack.h"

#define tomb_meret 50      // 50 minta = 10 ms @ 5 kHz
#define puffer_db  128     // 128 blokk = 1,28 s tartalék

// Egy keret: 2 bájt fejléc (AA 55) + 2 bájt sorszám + 50 minta = 104 bájt
typedef struct __attribute__((packed)) {
    uint8_t  m0, m1;
    uint16_t seq;
    uint16_t adat[tomb_meret];
} keret_t;

static keret_t puffer[puffer_db];
static uint8_t  szamlalo = 0;
static uint16_t blokk_szam = 0;        // minden kész blokk kap egyet, eldobás esetén is
static volatile uint32_t iras  = 0;    // core1 növeli: kész blokkok száma
static volatile uint32_t olvas = 0;    // core0 növeli: elküldött blokkok száma
static volatile uint32_t eldobott = 0;

// ---------- core1: mintavétel ----------
static bool adc_olvasas(struct repeating_timer *t) {
    puffer[iras % puffer_db].adat[szamlalo++] = adc_read();

    if (szamlalo >= tomb_meret) {
        szamlalo = 0;
        if (iras - olvas < puffer_db - 1) {      // mindig marad egy szabad hely
            keret_t *k = &puffer[iras % puffer_db];
            k->m0 = 0xAA;
            k->m1 = 0x55;
            k->seq = blokk_szam;
            __dmb();                              // a keret előbb látszódjon, mint az index
            iras = iras + 1;
        } else {
            eldobott++;                           // a sorszám hézaga jelzi a PC-nek
        }
        blokk_szam++;
    }
    return true;
}

void Core1_entry() {
    flash_safe_execute_core_init();   // engedi, hogy a core0 flash-t írjon

    adc_init();
    adc_gpio_init(26);
    adc_select_input(0);

    alarm_pool_t *pool = alarm_pool_create_with_unused_hardware_alarm(4);
    static struct repeating_timer timer;
    alarm_pool_add_repeating_timer_us(pool, -200, adc_olvasas, NULL, &timer);

    while (true) tight_loop_contents();
}

// ---------- core0: Bluetooth SPP ----------
static uint16_t rfcomm_cid = 0;
static uint8_t spp_service_buffer[150];
static btstack_packet_callback_registration_t hci_cb;
static btstack_timer_source_t poll_timer;

static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(channel); UNUSED(size);
    if (packet_type != HCI_EVENT_PACKET) return;

    switch (hci_event_packet_get_type(packet)) {
        case HCI_EVENT_PIN_CODE_REQUEST: {
            bd_addr_t addr;
            hci_event_pin_code_request_get_bd_addr(packet, addr);
            gap_pin_code_response(addr, "0000");
            break;
        }
        case RFCOMM_EVENT_INCOMING_CONNECTION:
            rfcomm_accept_connection(rfcomm_event_incoming_connection_get_rfcomm_cid(packet));
            break;

        case RFCOMM_EVENT_CHANNEL_OPENED:
            if (rfcomm_event_channel_opened_get_status(packet) == 0) {
                rfcomm_cid = rfcomm_event_channel_opened_get_rfcomm_cid(packet);
                olvas = iras;             // a régi, felgyülemlett adatot eldobjuk
            }
            break;

        case RFCOMM_EVENT_CHANNEL_CLOSED:
            rfcomm_cid = 0;
            break;

        case RFCOMM_EVENT_CAN_SEND_NOW:
            if (rfcomm_cid) {
                uint32_t elerheto = iras - olvas;
                if (elerheto) {
                    uint32_t idx = olvas % puffer_db;
                    uint32_t n = elerheto;
                    if (n > puffer_db - idx) n = puffer_db - idx;   // ne lépjen át a gyűrű végén

                    uint32_t max_blokk = rfcomm_get_max_frame_size(rfcomm_cid) / sizeof(puffer[0]);
                    if (max_blokk < 1) max_blokk = 1;
                    if (n > max_blokk) n = max_blokk;

                    __dmb();
                    if (rfcomm_send(rfcomm_cid, (uint8_t *)&puffer[idx],
                                    n * sizeof(puffer[0])) == 0) {
                        olvas = olvas + n;
                    }
                }
                if (iras != olvas) {
                    rfcomm_request_can_send_now_event(rfcomm_cid);
                }
            }
            break;
    }
}

// 5 ms-enként megnézzük, van-e új blokk a core1-től
static void poll_handler(btstack_timer_source_t *ts) {
    static uint32_t utolso = 0;
    uint32_t most = to_ms_since_boot(get_absolute_time());
    if (most - utolso >= 1000) {
        utolso = most;
        printf("eldobott: %lu, sorban: %lu\n",
               (unsigned long)eldobott, (unsigned long)(iras - olvas));
    }

    if (iras != olvas) {
        if (rfcomm_cid) {
            rfcomm_request_can_send_now_event(rfcomm_cid);
        } else {
            olvas = iras;             // nincs kapcsolat, eldobjuk
        }
    }
    btstack_run_loop_set_timer(ts, 5);
    btstack_run_loop_add_timer(ts);
}

static void spp_setup(void) {
    l2cap_init();
    rfcomm_init();
    rfcomm_register_service(packet_handler, 1, 0xffff);

    sdp_init();
    memset(spp_service_buffer, 0, sizeof(spp_service_buffer));
    spp_create_sdp_record(spp_service_buffer, sdp_create_service_record_handle(), 1, "ADC SPP");
    sdp_register_service(spp_service_buffer);

    gap_set_local_name("Pico2W ADC");
    gap_discoverable_control(1);
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    gap_ssp_set_auto_accept(1);
    gap_set_default_link_policy_settings(LM_LINK_POLICY_ENABLE_ROLE_SWITCH);  // sniff mód tiltása

    hci_cb.callback = &packet_handler;
    hci_add_event_handler(&hci_cb);

    poll_timer.process = &poll_handler;
    btstack_run_loop_set_timer(&poll_timer, 5);
    btstack_run_loop_add_timer(&poll_timer);

    hci_power_control(HCI_POWER_ON);
}

int main() {
    stdio_init_all();

    if (cyw43_arch_init()) {
        printf("cyw43 init hiba\n");
        return -1;
    }

    spp_setup();
    multicore_launch_core1(Core1_entry);   // a BT init után indítjuk

    btstack_run_loop_execute();            // innen nem tér vissza

    return 0;
}