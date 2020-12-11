#include <stdio.h>
#include <string.h>
#include "modbus.h"
#include "dcts.h"

/**
  * @defgroup modbus
  * @brief mudbus functions for communicate
  */

/*  "01 - Read Coils":                ('1',  'req_input', 2000, "BOOL",  1, "Q", "X", "Coil"),
    "02 - Read Input Discretes":      ('2',  'req_input', 2000, "BOOL",  1, "I", "X", "Input Discrete"),
    "03 - Read Holding Registers":    ('3',  'req_input',  125, "WORD", 16, "Q", "W", "Holding Register"),
    "04 - Read Input Registers":      ('4',  'req_input',  125, "WORD", 16, "I", "W", "Input Register"),
    "05 - Write Single coil":         ('5', 'req_output',    1, "BOOL",  1, "Q", "X", "Coil"),
    "06 - Write Single Register":     ('6', 'req_output',    1, "WORD", 16, "Q", "W", "Holding Register"),
   ?"15 - Write Multiple Coils":     ('15', 'req_output', 1968, "BOOL",  1, "Q", "X", "Coil"),
    "16 - Write Multiple Registers": ('16', 'req_output',  123, "WORD", 16, "Q", "W", "Holding Register")}
*/

/*========= GLOBAL VARIABLES ==========*/

/**
 * @brief htons for buffers lenths
 * @param word_numm - number uint16 words(uint8 * 2)
 * @return number of replaced world(uint16)
 * @ingroup modbus
 **/
u8 htons_buff(u16 *buff,u8 word_numm){
    u8 i;
    for (i = 0;i<word_numm;i++){
        buff[i] = htons(buff[i]);
    }
    return i;
}

/**
 * @brief Count crc16 for packet modbus
 * @param pckt - pointer to counted buffer
 * @param len  - length  packet with out two bytes crc
 * len type u16 for use not only modbus
 * @return crc
 * @ingroup modbus
 * */
u16 modbus_crc16(u8* pckt, u16 len){
    u16  result;
    u16 i, j;
    len = len > 254?254:len;
    result = 0xFFFF;
    for (i = 0; i < len; i++)  {
        result ^= pckt[i];
        for (j = 0; j < 8; j++) {
            if ((result & 0x01) == 1){
                result = (result >> 1) ^ 0xA001;
            }else{
                result >>= 1;
            }
        }
    }
    return result;
}

/**
 * @brief Control crc16 for packet modbus, count self and compare with tail
 * @param pckt - pointer to counted buffer
 * @param len - length  packet with two bytes len
 * len type u16 for use not only modbus
 * @return  1 - OK,\n
 *          0 - not compared
 * @ingroup modbus
 * */
u8 modbus_crc16_check(u8* pckt,u16 lenght){
    if (lenght < 3){
        return 0;
    }
    uint16_t test_1 = modbus_crc16 (pckt, lenght-2);
    uint16_t test_2 = *(u16*)(void*)(pckt + lenght - 2);
    if (modbus_crc16 (pckt, lenght-2) != *(u16*)(void*)(pckt + lenght - 2)) {
        return 0;
    }
    return 1;
}

/**
 * @brief Check modbus address in packet with device address
 * @param pckt - pointer to counted buffer
 * @param len - length  packet with two bytes len
 * @return  1 - for me,\n
 *          0 - not for me
 * @ingroup modbus
 * */
u8 modbus_packet_for_me(u8* pckt,u16 lenght){
    if (lenght){
        if ((pckt[0] != dcts.dcts_address)  &&
            (pckt[0] != MODBUS_BROADCAST_ADDRESS)){
            return 0;
        }else{
            return 1;
        }
    }else{
        return 0;
    }
}

/**
 * @brief modbus_tcp_packet
 * @param pckt - pointer to counted buffer
 * @param len - length incoming packet
 * @return length reply packet
 */
/*u16 modbus_tcp_packet (u8* pckt, u16 len){
    u16 reply_len;
    //add fake crc for modbus_rtu_packet
    *(u16*)(void*)(&pckt[len]) = modbus_crc16((u8*)(pckt+MODBUS_TCP_HEADER_SIZE),\
                                        len-MODBUS_TCP_HEADER_SIZE);
    reply_len = modbus_rtu_packet(&pckt[MODBUS_TCP_HEADER_SIZE],\
                                  len-MODBUS_TCP_HEADER_SIZE+2);
    if(reply_len>4){
        reply_len -= 2;//menos crc
        pckt[4] = (u8)(reply_len>>8);//to header mdb_tcp
        pckt[5] = (u8)(reply_len);//to header mdb_tcp
        reply_len = reply_len+MODBUS_TCP_HEADER_SIZE;//mdb_tcp header
    }
    return reply_len;
}*/

/**
 * @brief
 * 01-02 and 0x00 for read coils regs
 * 03 function for read regs
 * 04 function for read regs
 * 05 write one coil
 * 06 write one reg
 * 15 write coils
 * 16 write regs
 * @param pckt - pointer to counted buffer
 * @param len_in - length incoming packet
 * @return length reply packet
 * @ingroup modbus
 */
u16 modbus_rtu_packet (u8* pckt,u16 len_in){
    u16 len_reply;
    u8 error, function;
    u16 start_address, regs_numm, num_bit;
    len_reply = 0;
    error = 0;
    if (it_modbus_request_check(pckt, len_in)==1){
        function = pckt[1];
        /*if (sofi.vars.mdb_revers){
            //replace 4 and 3 function if need
            function = ((function == 3)?4:(function == 4?3:function));
        }
        if (sofi.vars.mdb_shift){    //for 3,4, 6, 16
            start_address=1;
        }else{
            start_address=0;
        }*/
        switch (function) {
        /*case 0:
        case 1:
        case 2:
        */
            /*read coil regs only for dinamic space coil to byte type
            in user space used one byte for one coil*/
            /*{
                u16 coil_number;
                coil_number = (u16)(pckt[2] << 8) + pckt[3];		// 2-3 bit start
                num_bit = (u16)(pckt[4] << 8) + pckt[5];		// 4-5num bit
                pool_id = modbus_dinamic_addr_check(coil_number,function,num_bit);
                if (pool_id >= 0){
                    base_address = modbus_dinamic_addr_get(pool_id);
                    if (base_address != NULL){
                        coil_number -= base_address->address;
                        len_reply = (u8)((num_bit+7)/8);
                        if (num_bit == 0) {
                            error = ILLEGAL_DATA_VALUE;
                        } else {
                            regs_access_t reg;
                            reg.flag = U8_REGS_FLAG;
                            u8 bit_n,byte_n;
                            for (u16 i=0; i<num_bit; i++){
                                osMutexWait(user_regs_access_mutex, portMAX_DELAY );{
                                    memcpy(&reg.value.op_u8,base_address->data + coil_number + i , 1);
                                }osMutexRelease(user_regs_access_mutex);
                                bit_n = i%8;
                                byte_n = (u8)(i/8);
                                if(reg.value.op_u8 & BIT(0)){
                                    *(u8 *)(pckt+3+byte_n)|=BIT(bit_n);
                                }else{
                                    *(u8 *)(pckt+3+byte_n)&=~BIT(bit_n);
                                }
                            }
                            if (!error) {
                                pckt[2] = (u8)len_reply;
                                len_reply += 5;
                            }
                        }
                    }
                }
            }
            break;*/
        case 3:
            /*should read only regs*/
            start_address = (u16)(pckt[2] << 8) + pckt[3];
            regs_numm = (u16)(pckt[4] << 8) + pckt[5];
            if (regs_numm < 1) {
                error = ILLEGAL_DATA_VALUE;
            }else if(start_address > MEAS_NUM*2){
                error = ILLEGAL_DATA_ADDRESS;
            }else if(!error){
                len_reply = (u8)(regs_numm << 1);
                pckt[2] = (u8)len_reply;
                dcts_mdb_t data = {0};
                for(uint8_t i = 0; i < regs_numm/2; i++){
                    data.f = dcts_meas[start_address/2 + i].value;
                    for(uint8_t byte_nmb = 0; byte_nmb < 4; byte_nmb++){
                        pckt[3+4*i+byte_nmb] = data.byte[byte_nmb];
                    }
                }
                len_reply += 5;
            }
            break;
        case 4:
            /*read writable regs*/
            start_address = (u16)(pckt[2] << 8) + pckt[3];
            regs_numm = (u16)(pckt[4] << 8) + pckt[5];
            if (regs_numm < 1) {
                error = ILLEGAL_DATA_VALUE;
            }else if(start_address > MEAS_NUM*2){
                error = ILLEGAL_DATA_ADDRESS;
            }else if(!error){
                len_reply = (u8)(regs_numm << 1);
                pckt[2] = (u8)len_reply;
                dcts_mdb_t data = {0};
                for(uint8_t i = 0; i < regs_numm/2; i++){
                    data.f = dcts_meas[start_address/2 + i].value;
                    for(uint8_t byte_nmb = 0; byte_nmb < 4; byte_nmb++){
                        pckt[3+4*i+byte_nmb] = (u8)data.byte[byte_nmb];
                    }
                }
                len_reply += 5;
            }
            break;
        //case 5:
            /*write bit (coil) only for dinamic space coil to byte type
            in user space used one byte for one coil*/
            /*{
                u16 coil_number,status;
                regs_access_t reg_temp;
                coil_number= (u16)((u16)pckt[2] << 8) + (u16)pckt[3];
                status  = (u16)((u16)pckt[4] << 8) + ((u16)pckt[5]);
                pool_id = modbus_dinamic_addr_check(coil_number, MDB_COILS_RW, 1);
                if (pool_id >= 0){
                    base_address = modbus_dinamic_addr_get(pool_id);
                    if (base_address!=NULL){
                        coil_number -= base_address->address;
                        */
                        /*for coil functions start address is byte address not mdb*/
                        /*reg_temp.flag = U8_REGS_FLAG;
                        if (!error) {
                            if(status == 0xff00){
                                reg_temp.value.op_u8 = BIT(0);
                            }else if (status == 0x0000){
                                reg_temp.value.op_u8 = 0;
                            }else{
                                error = ILLEGAL_DATA_VALUE;
                            }*/
                            /*writing*/
                            /*if (!error){
                                osMutexWait(user_regs_access_mutex, portMAX_DELAY );{
                                    memcpy(base_address->data + coil_number,&reg_temp.value.op_u8,1);
                                }osMutexRelease(user_regs_access_mutex);
                            }
                            len_reply = 8;*/
                            /*dont change first byte they will be in answear transaction*/
                        /*}
                    }
                }
            }
            break;
        case 6:*/
            /*write one word*/
            /*{
                regs_access_t reg;
                reg.flag = U16_REGS_FLAG;
                start_address += ((u16)(pckt[2] << 8) + pckt[3]);
                pool_id = modbus_dinamic_addr_check(start_address, MDB_HOLDING_REGS_RW, 1);
                if (pool_id >= 0){
                    base_address = modbus_dinamic_addr_get(pool_id);
                    start_address -= base_address->address;
                }else if (start_address >= SELF_MDB_ADDRESS_SPACE_START){
                    start_address -= SELF_MDB_ADDRESS_SPACE_START;
                }else{
                    error = ILLEGAL_DATA_ADDRESS;
                }
                if (!error){
                    reg.value.op_u16 = ((u16)(pckt[4] << 8) + pckt[5]);
                    len_reply = 8;
                    if(base_address==NULL){*/
                        /*write self regs*/
                        /*if (regs_set((start_address*2),reg)!=0) {
                            error = ILLEGAL_DATA_ADDRESS;
                        }
                    }else{
                        osMutexWait(user_regs_access_mutex, portMAX_DELAY );{
                            memcpy(base_address->data + start_address*2,&reg.value.op_u16,2);
                        }osMutexRelease(user_regs_access_mutex);
                    }
                }
                break;
            }
        case 15:*/
            /*write coils only for dinmaic space "coil to byte type"
            in user space used one byte for one coil*/
        /*{
            u8 bytes_num;
            u16 coil_number;
            coil_number = (u16)(pckt[2] << 8) + pckt[3];		// 2-3 bit start
            num_bit = (u16)(pckt[4] << 8) + pckt[5];		// 4-5num bit
            pool_id = modbus_dinamic_addr_check(coil_number, MDB_COILS_RW, num_bit);
            if (pool_id >= 0){
                base_address = modbus_dinamic_addr_get(pool_id);
                if (base_address!=NULL){
                    coil_number -= base_address->address;
                    bytes_num = pckt[6];
                    if (num_bit == 0) {
                        error = ILLEGAL_DATA_VALUE;
                    } else {
                        u16 write_bit = 0;
                        for (u16 i=0; i<bytes_num; i++){
                            regs_access_t reg;
                            reg.flag = U8_REGS_FLAG;
                            for(u8 j=0; (j<8) && num_bit; j++){
                                if(pckt[7+i] & BIT(j)){
                                    reg.value.op_u8 = BIT(0);
                                }else{
                                    reg.value.op_u8 = 0;
                                }*/
                                /*writing*/
                                /*osMutexWait(user_regs_access_mutex, portMAX_DELAY );{
                                    memcpy(base_address->data + coil_number + write_bit ,&reg.value.op_u8,1);
                                }osMutexRelease(user_regs_access_mutex);
                                write_bit++;
                                num_bit--;
                            }
                        }
                        if (!error) {
                            len_reply = 8;*/
                            /*dont change first byte they will be in answear transaction*/
                        /*}
                    }
                }else{
                    error = ILLEGAL_DATA_ADDRESS;
                }
            }else{
                error = ILLEGAL_DATA_ADDRESS;
            }
        }
        break;
        case 16:*/
            /*write words*/
            /*start_address += ((pckt[2] << 8) + pckt[3]);
            regs_numm = ((u16)(pckt[4] << 8) + pckt[5]);
            pool_id = modbus_dinamic_addr_check(start_address, MDB_HOLDING_REGS_RW, regs_numm);
            if (pool_id >= 0){
                base_address = modbus_dinamic_addr_get(pool_id);
                start_address -= base_address->address;
            }else if (start_address >= SELF_MDB_ADDRESS_SPACE_START){
                start_address -= SELF_MDB_ADDRESS_SPACE_START;
            }else{
                error = ILLEGAL_DATA_ADDRESS;
            }
            if (!error){
                htons_buff((u16*)(void*)(pckt+7), (u8)regs_numm);
                if (base_address==NULL){
                    if (regs_set_buffer(start_address*2, (u8*)(pckt+7), regs_numm*2)!=0) {
                        error = ILLEGAL_DATA_ADDRESS;
                    }
                }else{
                    osMutexWait(user_regs_access_mutex, portMAX_DELAY );{
                        memcpy(base_address->data + start_address*2, (u8*)(pckt+7), regs_numm*2);
                    }osMutexRelease(user_regs_access_mutex);
                }
                if (!error){
                    len_reply = 8;
                }
            }
            break;*/
        default:
            error = ILLEGAL_FUNCTION;
            break;
        }
        if(error){
            pckt[1] |= 0x80;
            pckt[2] = error;
            len_reply = 5;
        }
        *((u16*)(void*)(&pckt[len_reply-2])) = modbus_crc16 (pckt, len_reply-2);
    }
    return len_reply;
}
u8 genenerate_error_packet(u8* packet,u8 error_code){
    packet[0] = (u8)dcts.dcts_address;
    packet[1] |=0x80;
    packet[2] = error_code;
    *(u16 *)(void*)(&packet[3]) = modbus_crc16 (packet, 3);
    return 5;
}
u8 modbus_err_packet_type(u8 rtu_flag,u8* packet,u8 error_code){
    u8 len;
    if(rtu_flag == MODBUS_TCP_PACKET){
        genenerate_error_packet(&packet[6],error_code);
        len = 9;
        packet[5] = len;
    }else{
        genenerate_error_packet(packet,error_code);
        len = 5;
    }
    return len;
}
/**
 * @brief simply check packet for modbus,
 *  control crc and function number
 *
 * */

u8 it_modbus_simply_check(u8* pckt,u16 length){
    vu8 ret;
    ret = 0;
    if ((pckt[1]<=16)||(pckt[1]==0x41)){
        if (modbus_crc16_check(pckt, length)){
            return 1;
        }else{
            return 0;
        }
    }else{
        return 0;
    }
}
/**
 * @brief verbose check packet for modbus rtu, all used function
 * @return 1 if checked
 * */
int it_modbus_request_check(u8* buff,u16 length){
    int result=0;
    u8 first,command;
    //SOFI_ASSERT("modbus check buff error", (buff!=NULL));
    if(length <= 256){
        first = 0;
        command = buff[first+1];
        switch (command){
        case 0:
        case 1:
        case 2:
            {
                u16 bit_quant;
                bit_quant = (u16)((u16)buff[first+4] << 8) + ((u16)buff[first+5]);
                if(bit_quant<254){
                    if(it_modbus_simply_check(buff,8)){
                        result = 1;
                    }
                }
            }
            break;
        case 3:
        case 4:
            {
                u16 reg_quant;
                reg_quant = (u16)((u16)buff[first+4] << 8) + ((u16)buff[first+5]);
                if(reg_quant<127){
                    if(it_modbus_simply_check(buff,8)){
                        result = 1;
                    }
                }
            }
            break;
        case 5:
            {
                u16 status;
                status  = (u16)((u16)buff[first+4] << 8) + ((u16)buff[first+5]);
                if((status == 0xff00) || (status == 0x0000)){
                    if(it_modbus_simply_check(buff,8)){
                        result = 1;
                    }
                }
            }
        case 6:
            if(it_modbus_simply_check(buff,8)){
                result = 1;
            }
            break;
        case 15:
            {
                u16 coil_item,coil_quant;
                u8 coil_bytes;
                coil_item = (u16)((u16)buff[first+2] << 8) + (u16)buff[first+3];
                coil_quant = (u16)((u16)buff[first+4] << 8) + ((u16)buff[first+5]);
                coil_bytes = buff[first+6];
                if (coil_bytes && coil_quant){
                    if(it_modbus_simply_check(buff,9+coil_bytes)){
                        result = 1;
                    }
                }
            }
            break;
        case 16:
            {
                u16 reg_quant;
                u8 reg_quant_bytes;
                reg_quant = (u16)((u16)buff[first+4] << 8) + ((u16)buff[first+5]);
                reg_quant_bytes = buff[first+6];
                if ((reg_quant_bytes == reg_quant*2) && reg_quant && (reg_quant<126)){
                    if(it_modbus_simply_check(buff,9+reg_quant_bytes)){
                        result = 1;
                    }
                }
            }
            break;
        case 0x41:
            if(it_modbus_simply_check(buff,length)){
                result = 1;
            }
            break;
        default:
            result = -1;
            break;
        }
    }
    return result;
}
/**
 * @brief verbose check packet for modbus rtu, all used function
 *
 * */
int it_modbus_responde_check(u8* buff,u16 length){
    int result=0;
    u8 first,command;
    //SOFI_ASSERT("modbus check buff error", (buff!=NULL));
    if (length<=256){
        first = 0;
        //address = buff[first];
        command = buff[first+1];
        switch (command){
        case 0:
        case 1:
        case 2:
            {
                u8 byte_number;
                byte_number = buff[first+2];
                if(byte_number<251){
                    if(it_modbus_simply_check(buff,(byte_number + 5))){
                        result = 1;
                    }
                }
            }
            break;
        case 3:
        case 4:
            {
                u8 byte_number;
                byte_number = buff[first+2];
                if(byte_number<251){
                    if(it_modbus_simply_check(buff,(byte_number + 5))){
                        result = 1;
                    }
                }
            }
            break;
        case 5:
            {
                u16 status;
                status  = (u16)((u16)buff[first+4] << 8) + ((u16)buff[first+5]);
                if((status == 0xff00) || (status == 0x0000)){
                    if(it_modbus_simply_check(buff,8)){
                        result = 1;
                    }
                }
            }
        case 6:
            if(it_modbus_simply_check(buff,8)){
                result = 1;
            }
            break;
        case 15:
            if(it_modbus_simply_check(buff,8)){
                result = 1;
            }
            break;
        case 16:
            if(it_modbus_simply_check(buff,8)){
                result = 1;
            }
            break;
        case 0x41:
            result = 1;
            break;
        default:
            result = 0;
            break;
        }
    }
    return result;
}
/**
 * @brief verbose check packet for modbus, all used function
 * TODO!!! add asserts for buff and length
 * @return 1 if checked
 * @ingroup modbus
 * */
/*int it_modbus_tcp_full_check(u8* buff,u16 len){
    //add fake crc for it_modbus_full_check
    u8 temp_buff[256];
    memcpy(temp_buff,buff,len);
    *(u16*)(void*)(&temp_buff[len])=modbus_crc16((u8*)(temp_buff+MODBUS_TCP_HEADER_SIZE),\
                                               len-MODBUS_TCP_HEADER_SIZE);
    return (it_modbus_request_check(&temp_buff[MODBUS_TCP_HEADER_SIZE],\
                                 len-MODBUS_TCP_HEADER_SIZE+2)==1);
}*/

/**
 * @brief make packet from parametrs
 * @param slave_address simply will add to packet
 * @param function simply will add to packet
 * @param start_addr will add to packet with htons
 * @param reg_num will add to packet with htons for functions - 1,2,3,4,15,16
 * @param data_to_write pointer data from to for functions 15,16
 * @param packet where packet will safe
 * @return len made packet
 * @ingroup modbus
 * */
int modbus_make_packet (u8  slave_address,u8  function, u16 start_addr,
                         u16 reg_num, u8 * data_to_write, u8 * packet){
    int byte;
    byte = 0;
    union {
        u16 u16;
        u8  u8[2];
    } tmp;
    if(packet !=NULL){
        packet[byte++] = slave_address;
        packet[byte++] = function;
        tmp.u16 = htons(start_addr);
        packet[byte++] = tmp.u8[0];
        packet[byte++] = tmp.u8[1];
        if (function == 3 || function == 4 || function == 1 || function == 2){
            tmp.u16 = htons(reg_num);
            packet[byte++] = tmp.u8[0];
            packet[byte++] = tmp.u8[1];
        }else if((function == 16 ) && (reg_num <= 127)){
            if(data_to_write!=NULL){
                tmp.u16 = htons(reg_num);
                /*safe word num */
                packet[byte++] = tmp.u8[0];
                packet[byte++] = tmp.u8[1];
                u8 byte_numm = (u8)(reg_num << 1);
                /*safe byte num use only in 16 command*/
                packet[byte++] = byte_numm;
                u8 data_first = (u8)byte;
                /*safe data*/
                for (u8 i=0;i<byte_numm; i++){
                    packet[byte++] = data_to_write[i];
                }
                /*change to modbus indian*/
                htons_buff((u16*)(void*)(&packet[data_first]),(u8)reg_num);
            }else{
                byte = -1;
            }
        }else if((function == 15) && (reg_num <= 254)){
            /*set bits,all coil  regs have bit to byte addreses*/
            if(data_to_write!=NULL){
                u8 byte_count,bit_n,byte_n;
                tmp.u16 = htons(reg_num);
                /*safe word num */
                packet[byte++] = tmp.u8[0];
                packet[byte++] = tmp.u8[1];
                byte_count = (u8)((reg_num+7)/8);
                packet[byte++] = byte_count;
                for (u8 i=0;i<reg_num; i++){
                    bit_n = i%8;
                    byte_n = i/8;
                    if (data_to_write[i]&BIT(0)){
                        packet[byte + byte_n] |= BIT(bit_n);
                    }else{
                        packet[byte + byte_n] &= ~BIT(bit_n);
                    }
                }
                byte += byte_count;
            }else{
                byte = -1;
            }
        }else if(function == 6){
            /*write word*/
            if(data_to_write!=NULL){
                packet[byte++] = data_to_write[1];
                packet[byte++] = data_to_write[0];
            }else{
                byte = -1;
            }
        }else if(function == 5){
            /*set single bits,all coil  regs have bit to byte addreses*/
            u16 status;
            if (data_to_write[0]&BIT(0)){
                status = 0xff00;
            }else{
                status = 0x0000;
            }
            packet[byte++] = ((status>>8)&0xff);
            packet[byte++] = (status&0xff);
        }else{
            byte = -1;
        }
        /*add modbus crc*/
        if(byte > 0){
            *(u16*)(void*)(&packet[byte]) = modbus_crc16(packet,(u16)byte);
            byte +=2;
        }
    }else{
        byte = -1;
    }
    return byte;
}
/**
 * @brief add address space for modbus
 * @param mdb_address modbus space
 * @param space pointer for bring  to modbus space
 * @param len pointer to size space, will zero if mismatch occur
 * @return pointer to osPoolId
 * @ingroup modbus
 * */
/*osPoolId modbus_bind_address_space(u32 mdb_or_coil_addr,u8* space,u32 * len){
    return modbus_bind_address_space_by_command(mdb_or_coil_addr,space,len,ALL_MDB_CMD);
}*/
/**
 * @brief add address space for modbus
 * @param mdb_address modbus space
 * @param space pointer for bring  to modbus space
 * @param len pointer to size space in byte, will zero if mismatch occur
 * @param command - modbus command corresponding of request 01 - Coils rw,2 - Input Discretes ro,
 * 3 - Holding Registers rw,4 - Input Registers ro
 * @return pointer to osPoolId
 * @ingroup modbus
 * */
/*osPoolId modbus_bind_address_space_by_command(u32 mdb_or_coil_addr,u8* space,u32 * len, u8 command){
    if((modbus_dinamic_addr_check(mdb_or_coil_addr,command,(u16)*len) < 0) &&
            it_is_ram(space,*len)){
        dinamic_address_space_t * dinamic_address_space = NULL;
        dinamic_address_space = osPoolCAlloc(dinamic_address);
        if(dinamic_address_space!=NULL){
            dinamic_address_space->address = mdb_or_coil_addr;
            dinamic_address_space->len = *len;
            dinamic_address_space->data = space;
            dinamic_address_space->command = command;
        }else{
            *len=0;
        }
    }else{
        *len =0;
    }
    return dinamic_address;
}*/
/**
 * @brief check if address in dinamic space
 * @param mdb_addr - address for checking
 * @param command  - mdb command
 * @return  number for pool id if exist
 *          negative - if didt find
 * @ingroup modbus
 */
/*int modbus_dinamic_addr_check(u32 mdb_or_coil_addr,u8 command,u16 len){
    int result = -1;
    if(dinamic_address!=0){
        for(u16 i=0;i<dinamic_address->pool_sz;i++){
            dinamic_address_space_t * dinamic_address_space;
            dinamic_address_space = os_pool_get_by_index(dinamic_address,i);
            if(dinamic_address_space!=NULL){
                if((mdb_or_coil_addr >= dinamic_address_space->address) &&
                   ((mdb_or_coil_addr + len) <= (dinamic_address_space->address + dinamic_address_space->len)) &&
                   (command == dinamic_address_space->command || dinamic_address_space->command == 0xff)){
                    result = i;
                    break;
                }
            }
        }
    }
    return result;
}*/
/**
 * @brief calc new address and pointer for reading and writing address
 * @param reg_address - pionter will rewrite data to
 * @return pointer to dinamic_address_space_t
 * @ingroup modbus
 * @todo add description
 */
/*void * modbus_dinamic_addr_get(int pool_id){
    dinamic_address_space_t * res = NULL;
    if(dinamic_address!=0&&(pool_id>=0)){
        dinamic_address_space_t * dinamic_address_space;
        dinamic_address_space = os_pool_get_by_index(dinamic_address,(u32)pool_id);
        if(dinamic_address_space!=NULL){
            res = dinamic_address_space;
        }
    }
    return res;
}*/
