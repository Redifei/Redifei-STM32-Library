/*
 queue.h - queue header
 This file is part of Redifei STM32 Library.

 Redifei STM32 Library is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Redifei STM32 Library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Redifei STM32 Library.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

typedef struct {
  uint16_t head;
  uint16_t tail;
  uint16_t size;
  uint8_t* buf;
} Qtype_t;

#define Q_SIZE(HEAD, TAIL, LENGTH) ((HEAD + LENGTH) - TAIL) % LENGTH

#if 0
static void red_i2cEV_handler(struct red_i2cPort* this) {
  assert_param(IS_CONFIGED_I2C_PORT(this->setting));

  const red_i2c_hardware_t* i2cHW = this->setting->hw;
  red_i2c_userSetting_t* i2cUserSetting = this->setting->userSetting;

  uint8_t subaddress_sent = this->setting->deviceIDSent;
  static uint8_t final_stop;

  I2C_TypeDef* I2Cx = i2cHW->i2cPort;
  uint8_t bytes = this->setting->packet->size;

  uint8_t writing, reading;
  (this->setting->i2cDirection == I2C_Direction_Transmitter)
      ? (writing = true) : (reading = true);

  uint8_t SReg_1 = I2Cx->SR1;

  /* START */
  if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB) != RESET) {
    I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Current);
    I2C_AcknowledgeConfig(I2Cx, ENABLE);
    this->setting->index = 0;

    if (reading && subaddress_sent) {
      this->setting->deviceIDSent = true;
      if (bytes == 2)
        I2C_NACKPositionConfig(I2Cx, I2C_NACKPosition_Next);
      I2C_Send7bitAddress(I2Cx, this->setting->packet->addr, I2C_Direction_Receiver);
    }
    else
      I2C_Send7bitAddress(I2Cx, this->setting->packet->addr, I2C_Direction_Transmitter);
  }

  /* ADDR */
  else if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_ADDR) != RESET) {
    __DMB();
    if (bytes == 1 && reading && subaddress_sent) {
      I2C_AcknowledgeConfig(I2Cx, DISABLE);
      __DMB();
      (void) I2Cx->SR2;
      I2C_GenerateSTOP(I2Cx, ENABLE);
      final_stop = 1;
      I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);
    }
    else {
      (void) I2Cx->SR2;
      __DMB();
      if (bytes == 2 && reading && subaddress_sent) {
        I2C_AcknowledgeConfig(I2Cx, DISABLE);
        I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);
      }
      else if (bytes == 3 && reading && subaddress_sent)
        I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);
      else
        I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);
    }
  }

  /* BTF */
  else if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTF) != RESET) {
    final_stop = 1;

    if (reading && subaddress_sent) {
      if (bytes > 2) {
        I2C_AcknowledgeConfig(I2Cx, DISABLE);
        this->setting->packet->data[this->setting->index++] = I2C_ReceiveData(I2Cx);
        I2C_GenerateSTOP(I2Cx, ENABLE);
        final_stop = 1;
        this->setting->packet->data[this->setting->index++] = I2C_ReceiveData(I2Cx);
        I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);
      }
      else {
        if (final_stop)
          I2C_GenerateSTOP(I2Cx, ENABLE);
        else
          I2C_GenerateSTART(I2Cx, ENABLE);
        this->setting->packet->data[this->setting->index++] = I2C_ReceiveData(I2Cx);
        this->setting->packet->data[this->setting->index++] = I2C_ReceiveData(I2Cx);
        this->setting->index++;
      }
    }
    else {
      if (subaddress_sent || (writing)) {
        if (final_stop)
          I2C_GenerateSTOP(I2Cx, ENABLE);
        else
          I2C_GenerateSTART(I2Cx, ENABLE);
        this->setting->index++;
      }
      else {
        I2C_GenerateSTART(I2Cx, ENABLE);
        this->setting->deviceIDSent = true; // Read일땐, REP START후에 설정
      }
    }
    while ((I2Cx->CR1 & I2C_CR1_START) != RESET)
      ;
  }

  /* RXNE */
  else if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_RXNE) != RESET) {
    this->setting->packet->data[this->setting->index++] = I2C_ReceiveData(I2Cx);
    if (bytes == (this->setting->index + 3))
      I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);
    if (bytes == this->setting->index) {
      this->setting->index++;
    }
  }

  /* TXE */
  else if (I2C_GetFlagStatus(I2Cx, I2C_FLAG_TXE) != RESET) {
    if (subaddress_sent != false) {
      I2C_SendData(I2Cx, this->setting->packet->data[this->setting->index++]);
      if (bytes == this->setting->index)
        I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);
    }
    else {
      this->setting->index++;
      I2C_SendData(I2Cx, this->setting->packet->reg); //reg
/*
      if (writing)
        this->setting->deviceIDSent = true; // 추가된 부분/ Write는 REP START가 없으므로
*/
      if (reading || !bytes)
        I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);
    }
  }

  /* END */
  if (this->setting->index == this->setting->packet->size + 1) {
    this->setting->deviceIDSent = false;
    if (final_stop)
      I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);
    this->setting->busy = false;
  }
}
void i2c_ev_handler(void)
{
    static uint8_t subaddress_sent, final_stop; //flag to indicate if subaddess sent, flag to indicate final bus condition
    static int8_t index;        //index is signed -1==send the subaddress
    uint8_t SReg_1 = I2Cx->SR1; //read the status register here

    if (SReg_1 & 0x0001) {      //we just sent a start - EV5 in ref manual
        I2Cx->CR1 &= ~0x0800;  //reset the POS bit so ACK/NACK applied to the current byte
        I2C_AcknowledgeConfig(I2Cx, ENABLE);    //make sure ACK is on
        index = 0;              //reset the index
        if (reading && (subaddress_sent || 0xFF == reg)) {       //we have sent the subaddr
            subaddress_sent = 1;        //make sure this is set in case of no subaddress, so following code runs correctly
            if (bytes == 2)
                I2Cx->CR1 |= 0x0800;    //set the POS bit so NACK applied to the final byte in the two byte read
            I2C_Send7bitAddress(I2Cx, addr, I2C_Direction_Receiver);   //send the address and set hardware mode
        } else {                //direction is Tx, or we havent sent the sub and rep start
            I2C_Send7bitAddress(I2Cx, addr, I2C_Direction_Transmitter);        //send the address and set hardware mode
            if (reg != 0xFF)       //0xFF as subaddress means it will be ignored, in Tx or Rx mode
                index = -1;     //send a subaddress
        }
    } else if (SReg_1 & 0x0002) {       //we just sent the address - EV6 in ref manual
        // Read SR1,2 to clear ADDR
        __DMB(); // memory fence to control hardware
        if (bytes == 1 && reading && subaddress_sent) {     // we are receiving 1 byte - EV6_3
            I2C_AcknowledgeConfig(I2Cx, DISABLE);           // turn off ACK
            __DMB();
            (void)I2Cx->SR2;                                // clear ADDR after ACK is turned off
            I2C_GenerateSTOP(I2Cx, ENABLE);                 // program the stop
            final_stop = 1;
            I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);         // allow us to have an EV7
        } else {                    // EV6 and EV6_1
            (void)I2Cx->SR2;        // clear the ADDR here
            __DMB();
            if (bytes == 2 && reading && subaddress_sent) {     //rx 2 bytes - EV6_1
                I2C_AcknowledgeConfig(I2Cx, DISABLE);   //turn off ACK
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);        //disable TXE to allow the buffer to fill
            } else if (bytes == 3 && reading && subaddress_sent)       //rx 3 bytes
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);        //make sure RXNE disabled so we get a BTF in two bytes time
            else                //receiving greater than three bytes, sending subaddress, or transmitting
                I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);
        }
    } else if (SReg_1 & 0x004) {        //Byte transfer finished - EV7_2, EV7_3 or EV8_2
        final_stop = 1;
        if (reading && subaddress_sent) {     //EV7_2, EV7_3
            if (bytes > 2) {      //EV7_2
                I2C_AcknowledgeConfig(I2Cx, DISABLE);   //turn off ACK
                read_p[index++] = I2C_ReceiveData(I2Cx);    //read data N-2
                I2C_GenerateSTOP(I2Cx, ENABLE); //program the Stop
                final_stop = 1; //reuired to fix hardware
                read_p[index++] = I2C_ReceiveData(I2Cx);    //read data N-1
                I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE); //enable TXE to allow the final EV7
            } else {            //EV7_3
                if (final_stop)
                    I2C_GenerateSTOP(I2Cx, ENABLE);     //program the Stop
                else
                    I2C_GenerateSTART(I2Cx, ENABLE);    //program a rep start
                read_p[index++] = I2C_ReceiveData(I2Cx);    //read data N-1
                read_p[index++] = I2C_ReceiveData(I2Cx);    //read data N
                index++;        //to show job completed
            }
        } else {                //EV8_2, which may be due to a subaddress sent or a write completion
            if (subaddress_sent || (writing)) {
                if (final_stop)
                    I2C_GenerateSTOP(I2Cx, ENABLE);     //program the Stop
                else
                    I2C_GenerateSTART(I2Cx, ENABLE);    //program a rep start
                index++;        //to show that the job is complete
            } else {            //We need to send a subaddress
                I2C_GenerateSTART(I2Cx, ENABLE);        //program the repeated Start
                subaddress_sent = 1;    //this is set back to zero upon completion of the current task
            }
        }
        //we must wait for the start to clear, otherwise we get constant BTF
        while (I2Cx->CR1 & 0x0100) { ; }
    } else if (SReg_1 & 0x0040) {       //Byte received - EV7
        read_p[index++] = I2C_ReceiveData(I2Cx);
        if (bytes == (index + 3))
            I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);    //disable TXE to allow the buffer to flush so we can get an EV7_2
        if (bytes == index)       //We have completed a final EV7
            index++;            //to show job is complete
    } else if (SReg_1 & 0x0080) {       //Byte transmitted -EV8/EV8_1
        if (index != -1) {      //we dont have a subaddress to send
            I2C_SendData(I2Cx, write_p[index++]);
            if (bytes == index)   //we have sent all the data
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);        //disable TXE to allow the buffer to flush
        } else {
            index++;
            I2C_SendData(I2Cx, reg);       //send the subaddress
            if (reading || !bytes)      //if receiving or sending 0 bytes, flush now
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);        //disable TXE to allow the buffer to flush
        }
    }
    if (index == bytes + 1) {   //we have completed the current job
        //Completion Tasks go here
        //End of completion tasks
        subaddress_sent = 0;    //reset this here
        // I2Cx->CR1 &= ~0x0800;   //reset the POS bit so NACK applied to the current byte
        if (final_stop)  //If there is a final stop and no more jobs, bus is inactive, disable interrupts to prevent BTF
            I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);       //Disable EVT and ERR interrupts while bus inactive
        busy = 0;
    }
}
#endif
