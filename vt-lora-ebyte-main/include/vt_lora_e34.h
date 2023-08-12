#ifndef VT_LORA_E34_H
#define VT_LORA_E34_H

#include "vt_lora_base.h"

namespace vt {
    class lora_e34 : public detail::lora_base {
    private:
        struct lora_e3x_param params_ = {};

    public:
        lora_e34(HardwareSerial &hardware_serial,
                 pin_t digital_pin_M0,
                 pin_t digital_pin_M1)
                : detail::lora_base(hardware_serial,
                                    digital_pin_M0,
                                    digital_pin_M1) {}

        void set_param(uint16_t addr,
                       uint32_t baud,
                       enum LoRaParity parity,
                       uint32_t data_rate,
                       uint8_t channel,
                       enum LoRaTxPower tx_power,
                       bool enb_FEC,
                       bool save_params) {
            params_ = {};
            params_.head = save_params ? 0xc0 : 0xc2;
            params_.addh = addr & 0xff;
            params_.addl = (addr >> 8) & 0xff;
            params_.sped |= (static_cast<uint8_t>(parity) << 6);
            params_.sped |= (baud_to_lora(baud) << 3);
            params_.sped |= (rate_to_lora(data_rate));
            params_.chan |= channel;
            params_.option = 0b01000000 | (enb_FEC << 2) | static_cast<uint8_t>(tx_power);  // default Push-pull, WOR 250 ms

            if (!ser_op_) {
                hw_serial_->write(params_.head);
                hw_serial_->write(params_.addh);
                hw_serial_->write(params_.addl);
                hw_serial_->write(params_.sped);
                hw_serial_->write(params_.chan);
                hw_serial_->write(params_.option);
            }
#ifdef SoftwareSerial_h
            else {
                sw_serial_->write(params_.head);
                sw_serial_->write(params_.addh);
                sw_serial_->write(params_.addl);
                sw_serial_->write(params_.sped);
                sw_serial_->write(params_.chan);
                sw_serial_->write(params_.option);
            }
#endif
        }

        const lora_e3x_param &get_param() const { return params_; }

    private:
        static uint8_t rate_to_lora(uint32_t rate) {
            switch (rate) {
                case static_cast<uint32_t>(250000ul):
                    return 0b00;
                case static_cast<uint32_t>(1000000ul):
                    return 0b01;
                case static_cast<uint32_t>(2000000ul):
                    return 0b10;
                default:
                    return 0b00;
            }
        }

        static uint8_t baud_to_lora(uint32_t baud) {
            switch (baud) {
                case static_cast<uint32_t>(1200u):
                    return 0b000;
                case static_cast<uint32_t>(2400u):
                    return 0b001;
                case static_cast<uint32_t>(4800u):
                    return 0b010;
                case static_cast<uint32_t>(9600u):
                    return 0b011;
                case static_cast<uint32_t>(19200u):
                    return 0b100;
                case static_cast<uint32_t>(38400u):
                    return 0b101;
                case static_cast<uint32_t>(57600u):
                    return 0b110;
                case static_cast<uint32_t>(115200u):
                    return 0b111;
                default:
                    return 0b011;
            }
        }
    };
}

#endif //VT_LORA_E34_H
