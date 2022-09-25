/*
 * OpenCyphal Node to control NeoPixel LED stripes with an Adafruit Feather M4 CAN
 *
 * Hardware:
 *   - Adafruit Feather M4 CAN Express
 *   
 *   use this CAN library: https://github.com/adafruit/arduino-CAN
 *
 * Used Subject-IDs
 * 1005 - sub - Bit       - internal LED
 * 2010 - sub - Integer8  - light mode
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/
#include <CAN.h>

#include <107-Arduino-Cyphal.h>
#include <107-Arduino-MCP2515.h>
//#include <Adafruit_SleepyDog.h>
#include <Adafruit_NeoPixel.h>

/**************************************************************************************
 * DEFINES
 **************************************************************************************/

// Which pin on the Arduino is connected to the NeoPixels?
#define NEOPIXELPIN        13 // Raspberry Pi Pico

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 8 // Popular NeoPixel ring size

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace uavcan::node;
using namespace uavcan::_register;
using namespace uavcan::primitive::scalar;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static CanardNodeID const AUX_CONTROLLER_NODE_ID = 43;

static CanardPortID const ID_LED1                 = 1005U;
static CanardPortID const ID_LIGHT_MODE           = 2010U;

static int8_t const LIGHT_MODE_RED         =   1;
static int8_t const LIGHT_MODE_GREEN       =   2;
static int8_t const LIGHT_MODE_BLUE        =   3;
static int8_t const LIGHT_MODE_WHITE       =   4;
static int8_t const LIGHT_MODE_AMBER       =   5;
static int8_t const LIGHT_MODE_BLINK_RED   =  11;
static int8_t const LIGHT_MODE_BLINK_GREEN =  12;
static int8_t const LIGHT_MODE_BLINK_BLUE  =  13;
static int8_t const LIGHT_MODE_BLINK_WHITE =  14;
static int8_t const LIGHT_MODE_BLINK_AMBER =  15;
static int8_t const LIGHT_MODE_RUN_RED     = 101;
static int8_t const LIGHT_MODE_RUN_GREEN   = 102;
static int8_t const LIGHT_MODE_RUN_BLUE    = 103;
static int8_t const LIGHT_MODE_RUN_WHITE   = 104;
static int8_t const LIGHT_MODE_RUN_AMBER   = 105;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

bool transmitCanFrame(CanardFrame const & frame);
void onLed1_Received (CanardRxTransfer const &, Node &);
void onLightMode_Received(CanardRxTransfer const &, Node &);

/* Cyphal Service Requests */
void onList_1_0_Request_Received(CanardRxTransfer const &, Node &);
void onGetInfo_1_0_Request_Received(CanardRxTransfer const &, Node &);
void onAccess_1_0_Request_Received(CanardRxTransfer const &, Node &);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

Node node_hdl(transmitCanFrame, AUX_CONTROLLER_NODE_ID);

static const uavcan_node_GetInfo_Response_1_0 GET_INFO_DATA = {
    /// uavcan.node.Version.1.0 protocol_version
    {1, 0},
    /// uavcan.node.Version.1.0 hardware_version
    {1, 0},
    /// uavcan.node.Version.1.0 software_version
    {0, 1},
    /// saturated uint64 software_vcs_revision_id
    NULL,
    /// saturated uint8[16] unique_id
    {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
     0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff},
    /// saturated uint8[<=50] name
    {
        "generationmake.NeoPixelNode",
        strlen("generationmake.NeoPixelNode")},
};

static uint16_t updateinterval_light=250;

/* REGISTER ***************************************************************************/

static RegisterNatural8  reg_rw_uavcan_node_id                        ("uavcan.node.id",                         Register::Access::ReadWrite, AUX_CONTROLLER_NODE_ID,                  [&node_hdl](RegisterNatural8 const & reg) { node_hdl.setNodeId(reg.get()); });
static RegisterString    reg_ro_uavcan_node_description               ("uavcan.node.description",                Register::Access::ReadWrite, "L3X-Z AUX_CONTROLLER",                  nullptr);
static RegisterNatural16 reg_ro_uavcan_sub_led1_id                    ("uavcan.sub.led1.id",                     Register::Access::ReadOnly,  ID_LED1,                                 nullptr);
static RegisterString    reg_ro_uavcan_sub_led1_type                  ("uavcan.sub.led1.type",                   Register::Access::ReadOnly,  "uavcan.primitive.scalar.Bit.1.0",       nullptr);
static RegisterNatural16 reg_ro_uavcan_sub_lightmode_id               ("uavcan.sub.lightmode.id",                Register::Access::ReadOnly,  ID_LIGHT_MODE,                           nullptr);
static RegisterString    reg_ro_uavcan_sub_lightmode_type             ("uavcan.sub.lightmode.type",              Register::Access::ReadOnly,  "uavcan.primitive.scalar.Integer8.1.0",  nullptr);
static RegisterNatural16 reg_rw_aux_updateinterval_light              ("aux.updateinterval.light",               Register::Access::ReadWrite, updateinterval_light,                    [&node_hdl](RegisterNatural16 const & reg) { updateinterval_light=reg.get(); if(updateinterval_light<100) updateinterval_light=100; });
static RegisterList      reg_list;

Heartbeat_1_0<> hb;
Integer8_1_0<ID_LIGHT_MODE> uavcan_light_mode;

Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXELPIN, NEO_GRB);

void light_off()
{
  pixels.clear();
  pixels.show();
}
void light_green()
{
  pixels.fill(pixels.Color(0, 55, 0));
  pixels.show();
}
void light_red()
{
  pixels.fill(pixels.Color(55, 0, 0));
  pixels.show();
}
void light_blue()
{
  pixels.fill(pixels.Color(0, 0, 55));
  pixels.show();
}
void light_white()
{
  pixels.fill(pixels.Color(55, 55, 55));
  pixels.show();
}
void light_amber()
{
  pixels.fill(pixels.Color(55, 40, 0));
  pixels.show();
}

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
//  Watchdog.enable(1000);

  Serial.begin(115200);

  /* Setup LED pins and initialize */
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN_STANDBY, false); // turn off STANDBY
  pinMode(PIN_CAN_BOOSTEN, OUTPUT);
  digitalWrite(PIN_CAN_BOOSTEN, true); // turn on booster
  
 /* start the CAN bus at 250 kbps */
  if (!CAN.begin(250E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
  // register the receive callback
  CAN.onReceive(onReceive);

  /* Configure initial heartbeat */
  uavcan_light_mode.data.value = LIGHT_MODE_RUN_BLUE;

  hb.data.uptime = 0;
  hb = Heartbeat_1_0<>::Health::NOMINAL;
  hb = Heartbeat_1_0<>::Mode::INITIALIZATION;
  hb.data.vendor_specific_status_code = 0;

  /* Subscribe to the GetInfo request */
  node_hdl.subscribe<GetInfo_1_0::Request<>>(onGetInfo_1_0_Request_Received);
  reg_list.subscribe(node_hdl);
  reg_list.add(reg_rw_uavcan_node_id);
  reg_list.add(reg_ro_uavcan_node_description);
  reg_list.add(reg_ro_uavcan_sub_led1_id);
  reg_list.add(reg_ro_uavcan_sub_lightmode_id);
  reg_list.add(reg_ro_uavcan_sub_led1_type);
  reg_list.add(reg_ro_uavcan_sub_lightmode_type);
  reg_list.add(reg_rw_aux_updateinterval_light);
  /* Subscribe to the reception of Bit message. */
  node_hdl.subscribe<Bit_1_0<ID_LED1>>(onLed1_Received);
  node_hdl.subscribe<Integer8_1_0<ID_LIGHT_MODE>>(onLightMode_Received);

  /* Init Neopixel */
  pixels.begin();

  light_red();
  delay(100);
  light_amber();
  delay(100);
  light_green();
  delay(100);
  light_blue();
  delay(100);
  light_white();
  delay(100);
  light_off();
}

void loop()
{
  /* Process all pending OpenCyphal actions.
   */
  node_hdl.spinSome();

  /* Publish all the gathered data, although at various
   * different intervals.
   */
  static unsigned long prev_led = 0;
  static unsigned long prev_heartbeat = 0;
//  static unsigned long prev_led_toggle = 0;

  unsigned long const now = millis();

  /* light mode for neopixels */
  if((now - prev_led) > updateinterval_light)
  {
    static bool is_light_on = false;
    is_light_on = !is_light_on;
    static int running_light_counter = 0;
    running_light_counter ++;
    if(running_light_counter>=8) running_light_counter=0;

    if (uavcan_light_mode.data.value == LIGHT_MODE_RED)
      light_red();
    else if (uavcan_light_mode.data.value == LIGHT_MODE_GREEN)
      light_green();
    else if (uavcan_light_mode.data.value == LIGHT_MODE_BLUE)
      light_blue();
    else if (uavcan_light_mode.data.value == LIGHT_MODE_WHITE)
      light_white();
    else if (uavcan_light_mode.data.value == LIGHT_MODE_AMBER)
      light_amber();
    else if (uavcan_light_mode.data.value == LIGHT_MODE_RUN_RED||uavcan_light_mode.data.value == LIGHT_MODE_RUN_GREEN||uavcan_light_mode.data.value == LIGHT_MODE_RUN_BLUE||uavcan_light_mode.data.value == LIGHT_MODE_RUN_WHITE||uavcan_light_mode.data.value == LIGHT_MODE_RUN_AMBER)
    {
      if (uavcan_light_mode.data.value == LIGHT_MODE_RUN_RED)
      {
        pixels.setPixelColor(running_light_counter, pixels.Color(55, 0, 0));
        pixels.setPixelColor((running_light_counter+7)%8, pixels.Color(27, 0, 0));
        pixels.setPixelColor((running_light_counter+6)%8, pixels.Color(14, 0, 0));
        pixels.setPixelColor((running_light_counter+5)%8, pixels.Color(7, 0, 0));
        pixels.setPixelColor((running_light_counter+4)%8, pixels.Color(0, 0, 0));
        pixels.show();
      }
      else if (uavcan_light_mode.data.value == LIGHT_MODE_RUN_GREEN)
      {
        pixels.setPixelColor(running_light_counter, pixels.Color(0, 55, 0));
        pixels.setPixelColor((running_light_counter+7)%8, pixels.Color(0, 27, 0));
        pixels.setPixelColor((running_light_counter+6)%8, pixels.Color(0, 14, 0));
        pixels.setPixelColor((running_light_counter+5)%8, pixels.Color(0, 7, 0));
        pixels.setPixelColor((running_light_counter+4)%8, pixels.Color(0, 0, 0));
        pixels.show();
      }
      else if (uavcan_light_mode.data.value == LIGHT_MODE_RUN_BLUE)
      {
        pixels.setPixelColor(running_light_counter, pixels.Color(0, 0, 55));
        pixels.setPixelColor((running_light_counter+7)%8, pixels.Color(0, 0, 27));
        pixels.setPixelColor((running_light_counter+6)%8, pixels.Color(0, 0, 14));
        pixels.setPixelColor((running_light_counter+5)%8, pixels.Color(0, 0, 7));
        pixels.setPixelColor((running_light_counter+4)%8, pixels.Color(0, 0, 0));
        pixels.show();
      }
      else if (uavcan_light_mode.data.value == LIGHT_MODE_RUN_WHITE)
      {
        pixels.setPixelColor(running_light_counter, pixels.Color(55, 55, 55));
        pixels.setPixelColor((running_light_counter+7)%8, pixels.Color(27, 27, 27));
        pixels.setPixelColor((running_light_counter+6)%8, pixels.Color(14, 14, 14));
        pixels.setPixelColor((running_light_counter+5)%8, pixels.Color(7, 7, 7));
        pixels.setPixelColor((running_light_counter+4)%8, pixels.Color(0, 0, 0));
        pixels.show();
      }
      else if (uavcan_light_mode.data.value == LIGHT_MODE_RUN_AMBER)
      {
        pixels.setPixelColor(running_light_counter, pixels.Color(55, 40, 0));
        pixels.setPixelColor((running_light_counter+7)%8, pixels.Color(27, 20, 0));
        pixels.setPixelColor((running_light_counter+6)%8, pixels.Color(14, 10, 0));
        pixels.setPixelColor((running_light_counter+5)%8, pixels.Color(7, 5, 0));
        pixels.setPixelColor((running_light_counter+4)%8, pixels.Color(0, 0, 0));
        pixels.show();
      }
    }
    else if (is_light_on&&(uavcan_light_mode.data.value == LIGHT_MODE_BLINK_RED||uavcan_light_mode.data.value == LIGHT_MODE_BLINK_GREEN||uavcan_light_mode.data.value == LIGHT_MODE_BLINK_BLUE||uavcan_light_mode.data.value == LIGHT_MODE_BLINK_WHITE||uavcan_light_mode.data.value == LIGHT_MODE_BLINK_AMBER))
    {
      if (uavcan_light_mode.data.value == LIGHT_MODE_BLINK_GREEN)
        light_green();
      else if (uavcan_light_mode.data.value == LIGHT_MODE_BLINK_BLUE)
        light_blue();
      else if (uavcan_light_mode.data.value == LIGHT_MODE_BLINK_WHITE)
        light_white();
      else if (uavcan_light_mode.data.value == LIGHT_MODE_BLINK_AMBER)
        light_amber();
      else
        light_red();
    }
    else
      light_off();

    prev_led = now;
  }

  /* Update the heartbeat object */
  hb.data.uptime = millis() / 1000;
  hb = Heartbeat_1_0<>::Mode::OPERATIONAL;

  /* Publish the heartbeat once/second */
  if(now - prev_heartbeat > 1000) {
    node_hdl.publish(hb);
    prev_heartbeat = now;
  }

  /* Feed the watchdog to keep it from biting. */
//  Watchdog.reset();
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void onReceive(int packetSize) {
  // received a packet
  if (CAN.packetRtr()) {
  } 
  else {
    static CanardFrame frame;
    static char payload[8];
    frame.extended_can_id=CAN.packetId();
    frame.payload_size=packetSize;

    // only print packet data for non-RTR packets
    uint8_t i=0;
    while (CAN.available()) {
      payload[i]=(char)CAN.read();
      if(i<7) i++;
    }
    frame.payload=&payload;

    node_hdl.onCanFrameReceived(frame, micros());
  }
}

bool transmitCanFrame(CanardFrame const & frame)
{
  CAN.beginExtendedPacket(frame.extended_can_id);
  CAN.write((const uint8_t *)frame.payload, frame.payload_size);
  CAN.endPacket();
  return true;
}

void onReceiveBufferFull(CanardFrame const & frame)
{
  node_hdl.onCanFrameReceived(frame, micros());
}

void onLed1_Received(CanardRxTransfer const & transfer, Node & /* node_hdl */)
{
  Bit_1_0<ID_LED1> const uavcan_led1 = Bit_1_0<ID_LED1>::deserialize(transfer);

  if(uavcan_led1.data.value)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("Received Bit1: true");
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("Received Bit1: false");
  }
}

void onLightMode_Received(CanardRxTransfer const & transfer, Node & /* node_hdl */)
{
  uavcan_light_mode = Integer8_1_0<ID_LIGHT_MODE>::deserialize(transfer);
}

void onGetInfo_1_0_Request_Received(CanardRxTransfer const &transfer, Node & node_hdl)
{
  GetInfo_1_0::Response<> rsp = GetInfo_1_0::Response<>();
  rsp.data = GET_INFO_DATA;
  Serial.println("onGetInfo_1_0_Request_Received");
  node_hdl.respond(rsp, transfer.metadata.remote_node_id, transfer.metadata.transfer_id);
}
