
#include <SPI.h>
#include <MFRC522.h>


#define SS_PIN 21
#define RST_PIN 22
MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance.
uint8_t id;

bool LerID(){
  // Look for new cards
  if ( ! mfrc522.PICC_IsNewCardPresent()) 
  {
    return false; 
  }
  // Select one of the cards
  if ( ! mfrc522.PICC_ReadCardSerial()) 
  {
    return false;
  }
  //Show UID on serial monitor
  Serial.print("UID tag :");
  String content= "";

  for (byte i = 0; i < mfrc522.uid.size; i++) 
  {
     Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
     Serial.print(mfrc522.uid.uidByte[i], HEX);
     content.concat(String(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " "));
     content.concat(String(mfrc522.uid.uidByte[i], HEX));
  }
  Serial.println();
  Serial.print("Mensagem: ");
  content.toUpperCase();

  if (content.substring(1) == "89 8E ED 94") //change here the UID of the card/cards that you want to give access
  {
    Serial.println("TAG 1");
    id=1;
  }
  else if(content.substring(1) == "70 C0 CD 73")
  {
    Serial.println("TAG 2");
    id=2;
  }
  else if(content.substring(1) == "45 6E 20 79")
  {
    Serial.println("TAG 3");
    
    id=3;
  }

  
 else   {
    Serial.println("Tag Invalida");
    delay(3000);
    return false;
  }
  // Halt PICC
  mfrc522.PICC_HaltA();

  // Stop encryption on PCD
  mfrc522.PCD_StopCrypto1();

  return true;
 
} 

void IniciarRFID(){
  SPI.begin();      // Initiate  SPI bus
  mfrc522.PCD_Init();   // Initiate MFRC522
}