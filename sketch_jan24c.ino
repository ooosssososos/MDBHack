//------------------------------------------------------------------------------
// MDB Parser - Make MDB communications human readable
// 2014-04-15 dan@marginallyclever.com
// Copyright at end of file.
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------

#include "config.h"



//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------

byte msg_address;
byte msg_data[MAX_MSG_LEN];
int msg_count=0;




//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------

struct MDB_Byte {
  byte data;
  byte mode;
};


//------------------------------------------------------------------------------
// MDB Parser - Make MDB communications human readable
// 2014-04-15 dan@marginallyclever.com
// Copyright at end of file.
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------

// cashless device
#define CMD_CD_RESET           (0x10 & COMMAND_MASK)  // Self-reset
#define CMD_CD_SETUP           (0x11 & COMMAND_MASK)  // Setup
#define CMD_CD_POLL            (0x12 & COMMAND_MASK)  // Poll
#define CMD_CD_VEND            (0x13 & COMMAND_MASK)  // Vend
#define CMD_CD_READER          (0x14 & COMMAND_MASK)  // Reader
#define CMD_CD_REVALUE         (0x15 & COMMAND_MASK)  // Revalue
#define CMD_CD_EXTRA           (0x17 & COMMAND_MASK)  // Extra

// states for the cashless devices
#define CD_STATE_INACTIVE   (0)
#define CD_STATE_DIABLED    (1)
#define CD_STATE_ENABLED    (2)
#define CD_STATE_IDLE       (3)
#define CD_STATE_VEND       (4)
#define CD_STATE_REVALUE    (5)
#define CD_STATE_NEGVEND    (6)

//------------------------------------------------------------------------------
// METHODS
//z------------------------------------------------------------------------------

MDB_Byte make(int val){
  MDB_Byte b;
  b.data = val & 0b11111111;
  b.mode = val & (1 << 8);
  return b;
}

int state = 0;

void CD_parse(int id) {
  Serial.print(F("Cashless Device "));
  Serial.print(id);
  Serial.print(' ');

  switch(msg_address & COMMAND_MASK) {
  case CMD_CD_RESET  :  Serial.print(F("Self-reset"));
    ack();
    break;
  case CMD_CD_SETUP  :  Serial.print(F("Setup"));
    Serial.print((int)msg_data[0]);
    if((int)msg_data[0]) {
      ack();
      Serial.print("MINMAX");
    }else{
      
      state = 0;
      MDB_Byte data[8] = {make(0b1),make(1),make(0),make(1),make(1),make(0),make(2),make(0)};
      MDB_transmitData(data,8);
      Serial.print("SETUP");
    }
    break;
  case CMD_CD_POLL   :  Serial.print(F("Poll"));

    if(!state){
      state = 1;
      MDB_Byte data[3] = {make(3),make(5),make(5)};
      Serial.print("sent");
      MDB_transmitData(data,3);
    }else{
      ack();
     // MDB_Byte data[3] = {make(5),make(5),make(5)};
    //  Serial.print("sent approved");
    //  MDB_transmitData(data,3);
    }
    break;
  case CMD_CD_VEND   :  Serial.print(F("Vend"));
    switch(msg_data[0]) {
    case 0x00:  Serial.print(F(" request"));
      MDB_Byte data[3] = {make(5),make(0),make(5)};
      MDB_transmitData(data,3);
    break;
    case 0x01:  Serial.print(F(" cancel"));  break; 
      ack(); 
    case 0x02:  Serial.print(F(" success"));  break; 
      ack(); 
    case 0x03:  Serial.print(F(" failure"));  break;
    case 0x04:  Serial.print(F(" session complete"));  break; 
      ack();         
    case 0x05:  Serial.print(F(" cash sale"));  break;
    case 0x06:  Serial.print(F(" neg request"));  break;
    }
    break;
  case CMD_CD_READER :  Serial.print(F("Reader"));
    switch(msg_data[0]) {
    case 0x00:  Serial.print(F(" disable"));  break;
    case 0x01:  Serial.print(F(" enable"));  break;
    case 0x02:  Serial.print(F(" cancel"));  break;
    case 0x03:  Serial.print(F(" data entry response"));  break;
    }
    break;
  case CMD_CD_REVALUE:  Serial.print(F("Revalue"));
    switch(msg_data[0]) {
    case 0x00:  Serial.print(F(" request"));  break;
    case 0x01:  Serial.print(F(" limit request"));  break;
    }
    break;
  case CMD_CD_EXTRA  :  Serial.print(F("Extra"));
    switch(msg_data[0]) {
    case 0x00:  Serial.print(F(" request ID"));
      
      state = 0;
      MDB_Byte data[30] = {make(9),make(0),make(0),make(1),make(0),make(0),make(0),make(0),make(0),make(0),make(1),make(0),make(0),make(0),make(0),make(1),
      make(0),make(0),make(0),make(0),make(0),make(0),make(1),make(0),make(0),make(0),make(0),make(1),
      make(0),make(5),};
      MDB_transmitData(data,30);
    
    break;
    case 0x01:  Serial.print(F(" read user file"));  break;
    case 0x02:  Serial.print(F(" write user file"));  break;
    case 0x03:  Serial.print(F(" write time/date"));  break;
    case 0x04:  Serial.print(F(" optional feature enabled"));  break;
    case 0xFA:  Serial.print(F(" FTL REQ2RCV"));  break;
    case 0xFB:  Serial.print(F(" FTL RETRY/DENY"));  break;
    case 0xFC:  Serial.print(F(" FTL SEND BLOCK"));  break;
    case 0xFD:  Serial.print(F(" FTL OK"));  break;
    case 0xFE:  Serial.print(F(" DTL REQ2SND"));  break;
    case 0xFF:  Serial.print(F(" diagnnostics"));  break;
    }
    break;
  default:
    Serial.print(F("CMD #"));
    Serial.print((int)(msg_address & COMMAND_MASK));
    break;
  }
}


//------------------------------------------------------------------------------
// MDB Parser - Make MDB communications human readable
// 2014-04-15 dan@marginallyclever.com
// Copyright at end of file.
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------

void AVD_parse() {
  Serial.print(F("Age verification device "));
}





/**
 * This file is part of MDB Parser.
 *
 * MDB Parser is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * MDB Parser is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with MDB Parser.  If not, see <http://www.gnu.org/licenses/>.
 */


/**
 * This file is part of MDB Parser.
 *
 * MDB Parser is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * MDB Parser is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with MDB Parser.  If not, see <http://www.gnu.org/licenses/>.
 */


/**
 * This file is part of MDB Parser.
 *
 * MDB Parser is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * MDB Parser is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with MDB Parser.  If not, see <http://www.gnu.org/licenses/>.
 */
void parse_message() {
  switch(msg_address & ADDRESS_MASK) {
  case ADDRESS_CD1      :  CD_parse(1);  break;
  
  default:
    Serial.print(F("ADR #"));
    Serial.print((int)(msg_address & ADDRESS_MASK));
    Serial.print(F(" CMD #"));
    Serial.print((int)(msg_address & COMMAND_MASK));
    break;
  }
}


/**
 * Displays byte in binary.  Useful for debugging comms at the start.
 */
void echo_data(char data) {
  char a;
  for(int i=0x80;i;i>>=1) {
    a = data & i;
    Serial.print(a?'1':'0');
  }
  //Serial.print(data);
}

void setup() {
  
  Serial.begin (115200);  // debugging prints
  Serial2.begin (9600, SERIAL_8N1, true);  // 9 bit mode
}




byte calculate_checksum() {
  byte value = msg_address;
  for(int i=0;i<msg_count;++i) {
    value+=msg_data[i];
  }
  return value;
}



void MDB_getByte(struct MDB_Byte* mdbb) {
  int b;
  char tmpstr[7];

  b = 0;
  b = (int)Serial2.read();
  memcpy (mdbb, &b, 2);
  
  /*if (gDebug > 1) {
    if (b.mode) sprintf(tmpstr, ">%.2x*", b.data);
    else        sprintf(tmpstr, ">%.2x", b.data);
    dbg_print(tmpstr);
  }*/
}


byte MDB_checksumValidate(byte addr,byte* data, byte length) {
  byte sum = addr;
  
  for (int i=0; i < (length-1); i++)
    sum += data[i];

  if (data[length-1] == sum)
    return 1;
  else
    return 0;
}



void loop() {
  if(Serial2.available()) {
    struct MDB_Byte parsed;
    MDB_getByte(&parsed);
    char data,mode;
    mode = parsed.mode;
    data = parsed.data;

    if(mode!=0) {
      // address byte is start of a new message block
      msg_count=0;
      msg_address = data;
      Serial.print("address");
    } else {
      // data or checksum
      if(msg_count<MAX_MSG_LEN) {
        msg_data[msg_count++]=data;
      }
      Serial.print((int)calculate_checksum());
      Serial.print(", ");
      Serial.print((int)data);
      if(MDB_checksumValidate(msg_address,msg_data, msg_count)) {
        
        // TODO could the checksum be received prematurely by accident?
        
        parse_message();
        Serial.print("\n");
      }
    }
  }
  
  //if(Serial.available()) {
  //  MDB.write(Serial.read());
  //}
}


byte MDB_checksumGenerate(struct MDB_Byte* data, byte length) {
  byte sum = 0;
  for (int i=0; i < (length); i++)
    sum += data[i].data;

  return sum;
}


#define sACK 0x100
#define rACK 0x00
#define RET 0xAA
#define sNAK 0x1FF
#define rNAK 0xFF

#define rRETRANSMIT 0
#define rCONTINUE 1
#define rUNKNOWN 2

byte MDB_transmitData(struct MDB_Byte* data, byte length) {
  int transmitState;
  struct MDB_Byte mdbb;
  int tx;
  char tmpstr[7];
  
  do
  {
    mdbb.data = MDB_checksumGenerate(data, length);
    mdbb.mode = 0x1;

    tx = 0;
    memcpy(&tx, &mdbb, 2);

    for (int i=0; i < length; i++)
      MDB_write(data[i].data);

    MDB_write(tx); //chk

    while ( !Serial2.available() );
    
    MDB_getByte(&mdbb);
    
    if (mdbb.data == rNAK || mdbb.data == RET) {
      transmitState = rRETRANSMIT;
    } else if (mdbb.data == rACK) {
      transmitState = rCONTINUE;
    } else {
      transmitState = rUNKNOWN;
    }
    if(transmitState == rRETRANSMIT){
      Serial.print("RETRANSMIT");
    }
  } while (transmitState == rRETRANSMIT);
  //if (debug) dbg_println(")");
  
  return transmitState;
}

void ack(){
  Serial2.write9bit(1<<8);
}


void MDB_write(int data) {
  char tmpstr[7];
  struct MDB_Byte b;
  
  memcpy(&b, &data, 2);
  
  
   Serial2.write9bit( (((int)b.data) |  (b.mode & 1) << 8) & 0b111111111);
  
  /*if (gDebug > 1) {
    if (b.mode) sprintf(tmpstr, ">%.2x*", b.data);
    else        sprintf(tmpstr, ">%.2x", b.data);
    dbg_print(tmpstr);
  }*/
}


/**
 * This file is part of MDB Parser.
 *
 * MDB Parser is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * MDB Parser is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with MDB Parser.  If not, see <http://www.gnu.org/licenses/>.
 */
