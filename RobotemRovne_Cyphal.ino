/**
 * Firmware for the Controller for the Robotem Rovne Contest, based on the CyphalPicoBase-CAN (https://github.com/generationmake/CyphalPicoBase-CAN)
 *
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2025 generationmake.
 * Author: Bernhard Mayer <bernhard@generationmake.de>
 * Contributors: https://github.com/generationmake/RobotemRovne-Cyphal/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>

#include <107-Arduino-Cyphal.h>
#include <107-Arduino-Cyphal-Support.h>

#include <107-Arduino-MCP2515.h>
#include <107-Arduino-littlefs.h>
#include <107-Arduino-24LCxx.hpp>

#define DBG_ENABLE_ERROR
#define DBG_ENABLE_WARNING
#define DBG_ENABLE_INFO
#define DBG_ENABLE_DEBUG
//#define DBG_ENABLE_VERBOSE
#include <107-Arduino-Debug.hpp>

#undef max
#undef min
#include <algorithm>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace uavcan::node;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static uint8_t const EEPROM_I2C_DEV_ADDR = 0x50;

static int const MCP2515_CS_PIN     = 17;
static int const MCP2515_INT_PIN    = 20;
static int const LED_2_PIN          = 21; /* GP21 */
static int const LED_3_PIN          = 22; /* GP22 */
static int const ANALOG_PIN         = 26;
static int const ANALOG_INPUT_0_PIN = 27;
static int const ANALOG_INPUT_1_PIN = 28;
  #define TFT_CS         6
  #define TFT_RST        9 // Or set to -1 and connect to Arduino RESET pin
  #define TFT_DC         7

static SPISettings const MCP2515x_SPI_SETTING{10*1000*1000UL, MSBFIRST, SPI_MODE0};

static uint16_t const UPDATE_PERIOD_HEARTBEAT_ms = 1000;

static uint32_t const WATCHDOG_DELAY_ms = 1000;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void onReceiveBufferFull(CanardFrame const & frame);
ExecuteCommand::Response_1_1 onExecuteCommand_1_1_Request_Received(ExecuteCommand::Request_1_1 const &);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

Adafruit_ST7735 tft = Adafruit_ST7735(&SPI1, TFT_CS, TFT_DC, TFT_RST);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

DEBUG_INSTANCE(80, Serial);

ArduinoMCP2515 mcp2515([]() { digitalWrite(MCP2515_CS_PIN, LOW); },
                       []() { digitalWrite(MCP2515_CS_PIN, HIGH); },
                       [](uint8_t const d) { return SPI.transfer(d); },
                       micros,
                       onReceiveBufferFull,
                       nullptr,
                       [](MCP2515::EFLG const err_flag) { DBG_ERROR("MCP2515::OnError, error code = \"%s\"", MCP2515::toStr(err_flag)); },
                       [](MCP2515::EFLG const err_flag) { DBG_ERROR("MCP2515::OnWarning, warning code = \"%s\"", MCP2515::toStr(err_flag)); });

cyphal::Node::Heap<cyphal::Node::DEFAULT_O1HEAP_SIZE> node_heap;
cyphal::Node node_hdl(node_heap.data(), node_heap.size(), micros, [] (CanardFrame const & frame) { return mcp2515.transmit(frame); });

cyphal::Publisher<Heartbeat_1_0> heartbeat_pub = node_hdl.create_publisher<Heartbeat_1_0>(1*1000*1000UL /* = 1 sec in usecs. */);
cyphal::Publisher<uavcan::primitive::scalar::Real32_1_0> input_voltage_pub;
cyphal::Publisher<uavcan::primitive::scalar::Real32_1_0> internal_temperature_pub;
cyphal::Publisher<uavcan::primitive::scalar::Integer16_1_0> analog_input_0_pub;
cyphal::Publisher<uavcan::primitive::scalar::Integer16_1_0> analog_input_1_pub;
cyphal::Publisher<uavcan::primitive::scalar::Integer16_1_0> motor_0_pwm_pub;
cyphal::Publisher<uavcan::primitive::scalar::Integer16_1_0> motor_1_pwm_pub;

cyphal::Subscription led_subscription;

cyphal::Subscription em_stop_subscription;

cyphal::ServiceServer execute_command_srv = node_hdl.create_service_server<ExecuteCommand::Request_1_1, ExecuteCommand::Response_1_1>(2*1000*1000UL, onExecuteCommand_1_1_Request_Received);

bool status_em_stop = 0;

/* LITTLEFS/EEPROM ********************************************************************/

static EEPROM_24LCxx eeprom(EEPROM_24LCxx_Type::LC64,
                            EEPROM_I2C_DEV_ADDR,
                            [](size_t const dev_addr) { Wire.beginTransmission(dev_addr); },
                            [](uint8_t const data) { Wire.write(data); },
                            []() { return Wire.endTransmission(); },
                            [](uint8_t const dev_addr, size_t const len) -> size_t { return Wire.requestFrom(dev_addr, len); },
                            []() { return Wire.available(); },
                            []() { return Wire.read(); });

static littlefs::FilesystemConfig filesystem_config
  (
    +[](const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size) -> int
    {
      eeprom.read_page((block * c->block_size) + off, (uint8_t *)buffer, size);
      return LFS_ERR_OK;
    },
    +[](const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size) -> int
    {
      eeprom.write_page((block * c->block_size) + off, (uint8_t const *)buffer, size);
      return LFS_ERR_OK;
    },
    +[](const struct lfs_config *c, lfs_block_t block) -> int
    {
      for(size_t off = 0; off < c->block_size; off += eeprom.page_size())
        eeprom.fill_page((block * c->block_size) + off, 0xFF);
      return LFS_ERR_OK;
    },
    +[](const struct lfs_config *c) -> int
    {
      return LFS_ERR_OK;
    },
    eeprom.page_size(),
    eeprom.page_size(),
    (eeprom.page_size() * 4), /* littlefs demands (erase) block size to exceed read/prog size. */
    eeprom.device_size() / (eeprom.page_size() * 4),
    500,
    eeprom.page_size(),
    eeprom.page_size()
  );
static littlefs::Filesystem filesystem(filesystem_config);

#if __GNUC__ >= 11
cyphal::support::platform::storage::littlefs::KeyValueStorage kv_storage(filesystem);
#endif /* __GNUC__ >= 11 */

/* REGISTER ***************************************************************************/

static uint16_t     node_id                      = std::numeric_limits<uint16_t>::max();
static CanardPortID port_id_input_voltage        = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_led1                 = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_internal_temperature = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_analog_input0        = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_analog_input1        = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_em_stop              = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_motor0_pwm           = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_motor1_pwm           = std::numeric_limits<CanardPortID>::max();

static uint16_t update_period_ms_inputvoltage        =  3*1000;
static uint16_t update_period_ms_internaltemperature = 10*1000;
static uint16_t update_period_ms_analoginput0        =     500;
static uint16_t update_period_ms_analoginput1        =     500;

static std::string node_description{"CyphalPicoBase/CAN"};

#if __GNUC__ >= 11

const auto node_registry = node_hdl.create_registry();

const auto reg_rw_cyphal_node_id                            = node_registry->expose("cyphal.node.id",                           {true}, node_id);
const auto reg_rw_cyphal_node_description                   = node_registry->expose("cyphal.node.description",                  {true}, node_description);
const auto reg_rw_cyphal_pub_inputvoltage_id                = node_registry->expose("cyphal.pub.inputvoltage.id",               {true}, port_id_input_voltage);
const auto reg_ro_cyphal_pub_inputvoltage_type              = node_registry->route ("cyphal.pub.inputvoltage.type",             {true}, []() { return "uavcan.primitive.scalar.Real32.1.0"; });
const auto reg_rw_cyphal_pub_internaltemperature_id         = node_registry->expose("cyphal.pub.internaltemperature.id",        {true}, port_id_internal_temperature);
const auto reg_ro_cyphal_pub_internaltemperature_type       = node_registry->route ("cyphal.pub.internaltemperature.type",      {true}, []() { return "uavcan.primitive.scalar.Real32.1.0"; });
const auto reg_rw_cyphal_pub_analoginput0_id                = node_registry->expose("cyphal.pub.analoginput0.id",               {true}, port_id_analog_input0);
const auto reg_ro_cyphal_pub_analoginput0_type              = node_registry->route ("cyphal.pub.analoginput0.type",             {true}, []() { return "uavcan.primitive.scalar.Integer16.1.0"; });
const auto reg_rw_cyphal_pub_analoginput1_id                = node_registry->expose("cyphal.pub.analoginput1.id",               {true}, port_id_analog_input1);
const auto reg_ro_cyphal_pub_analoginput1_type              = node_registry->route ("cyphal.pub.analoginput1.type",             {true}, []() { return "uavcan.primitive.scalar.Integer16.1.0"; });
const auto reg_rw_cyphal_sub_led1_id                        = node_registry->expose("cyphal.sub.led1.id",                       {true}, port_id_led1);
const auto reg_ro_cyphal_sub_led1_type                      = node_registry->route ("cyphal.sub.led1.type",                     {true}, []() { return "uavcan.primitive.scalar.Bit.1.0"; });
const auto reg_rw_cyphal_sub_em_stop_id                     = node_registry->expose("cyphal.sub.em_stop.id",                    {true}, port_id_em_stop);
const auto reg_ro_cyphal_sub_em_stop_type                   = node_registry->route ("cyphal.sub.em_stop.type",                  {true}, []() { return "uavcan.primitive.scalar.Bit.1.0"; });
const auto reg_rw_cyphal_pub_motor0_pwm_id                  = node_registry->expose("cyphal.pub.motor0pwm.id",                  {true}, port_id_motor0_pwm);
const auto reg_ro_cyphal_pub_motor0_pwm_type                = node_registry->route ("cyphal.pub.motor0pwm.type",                {true}, []() { return "uavcan.primitive.scalar.Integer16.1.0"; });
const auto reg_rw_cyphal_pub_motor1_pwm_id                  = node_registry->expose("cyphal.pub.motor1pwm.id",                  {true}, port_id_motor1_pwm);
const auto reg_ro_cyphal_pub_motor1_pwm_type                = node_registry->route ("cyphal.pub.motor1pwm.type",                {true}, []() { return "uavcan.primitive.scalar.Integer16.1.0"; });
const auto reg_rw_pico_update_period_ms_inputvoltage        = node_registry->expose("pico.update_period_ms.inputvoltage",        {true}, update_period_ms_inputvoltage);
const auto reg_rw_pico_update_period_ms_internaltemperature = node_registry->expose("pico.update_period_ms.internaltemperature", {true}, update_period_ms_internaltemperature);
const auto reg_rw_pico_update_period_ms_analoginput0        = node_registry->expose("pico.update_period_ms.analoginput0",        {true}, update_period_ms_analoginput0);
const auto reg_rw_pico_update_period_ms_analoginput1        = node_registry->expose("pico.update_period_ms.analoginput1",        {true}, update_period_ms_analoginput1);

#endif /* __GNUC__ >= 11 */

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(115200);
  // while(!Serial) { } /* only for debug */
  delay(1000);

  SPI1.setTX(11);
  SPI1.setSCK(10);

  Debug.prettyPrintOn(); /* Enable pretty printing on a shell. */

  /* LITTLEFS/EEPROM ********************************************************************/
  Wire.begin();
  Wire.setClock(400*1000UL); /* Set fast mode. */

  if (!eeprom.isConnected()) {
    DBG_ERROR("Connecting to EEPROM failed.");
    return;
  }
  Serial.println(eeprom);

  if (auto const err_mount = filesystem.mount(); err_mount.has_value()) {
    DBG_ERROR("Mounting failed with error code %d", static_cast<int>(err_mount.value()));
    (void)filesystem.format();
  }

  if (auto const err_mount = filesystem.mount(); err_mount.has_value()) {
    DBG_ERROR("Mounting failed again with error code %d", static_cast<int>(err_mount.value()));
    return;
  }

#if __GNUC__ >= 11
  auto const rc_load = cyphal::support::load(kv_storage, *node_registry);
  if (rc_load.has_value()) {
    DBG_ERROR("cyphal::support::load failed with %d", static_cast<int>(rc_load.value()));
    return;
  }
#endif /* __GNUC__ >= 11 */

  (void)filesystem.unmount();

  /* If the node ID contained in the register points to an undefined
   * node ID, assign node ID 0 to this node.
   */
  if (node_id > CANARD_NODE_ID_MAX)
    node_id = 0;
  node_hdl.setNodeId(static_cast<CanardNodeID>(node_id));

  if (port_id_input_voltage != std::numeric_limits<CanardPortID>::max())
    input_voltage_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Real32_1_0>(port_id_input_voltage, 1*1000*1000UL /* = 1 sec in usecs. */);

  if (port_id_led1 != std::numeric_limits<CanardPortID>::max())
    led_subscription = node_hdl.create_subscription<uavcan::primitive::scalar::Bit_1_0>(
      port_id_led1,
      [] (uavcan::primitive::scalar::Bit_1_0 const & msg)
      {
        if(msg.value)
          digitalWrite(LED_BUILTIN, HIGH);
        else
          digitalWrite(LED_BUILTIN, LOW);
      });

  if (port_id_internal_temperature != std::numeric_limits<CanardPortID>::max())
    internal_temperature_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Real32_1_0>(port_id_internal_temperature, 1*1000*1000UL /* = 1 sec in usecs. */);

  if (port_id_analog_input0 != std::numeric_limits<CanardPortID>::max())
    analog_input_0_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Integer16_1_0>(port_id_analog_input0, 1*1000*1000UL /* = 1 sec in usecs. */);
  if (port_id_analog_input1 != std::numeric_limits<CanardPortID>::max())
    analog_input_1_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Integer16_1_0>(port_id_analog_input1, 1*1000*1000UL /* = 1 sec in usecs. */);
  if (port_id_motor0_pwm != std::numeric_limits<CanardPortID>::max())
    motor_0_pwm_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Integer16_1_0>(port_id_motor0_pwm, 1*1000*1000UL /* = 1 sec in usecs. */);
  if (port_id_motor1_pwm != std::numeric_limits<CanardPortID>::max())
    motor_1_pwm_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Integer16_1_0>(port_id_motor1_pwm, 1*1000*1000UL /* = 1 sec in usecs. */);
  if (port_id_em_stop != std::numeric_limits<CanardPortID>::max())
    em_stop_subscription = node_hdl.create_subscription<uavcan::primitive::scalar::Bit_1_0>(
      port_id_em_stop,
      [](uavcan::primitive::scalar::Bit_1_0 const & msg)
      {
        status_em_stop = msg.value;
      });


    /* set factory settings */
    if(update_period_ms_inputvoltage==0xFFFF)        update_period_ms_inputvoltage=3*1000;
    if(update_period_ms_internaltemperature==0xFFFF) update_period_ms_internaltemperature=10*1000;
    if(update_period_ms_analoginput0==0xFFFF)        update_period_ms_analoginput0=500;
    if(update_period_ms_analoginput1==0xFFFF)        update_period_ms_analoginput1=500;

  /* NODE INFO **************************************************************************/
  static const auto node_info = node_hdl.create_node_info
  (
    /* cyphal.node.Version.1.0 protocol_version */
    1, 0,
    /* cyphal.node.Version.1.0 hardware_version */
    1, 0,
    /* cyphal.node.Version.1.0 software_version */
    0, 1,
    /* saturated uint64 software_vcs_revision_id */
#ifdef CYPHAL_NODE_INFO_GIT_VERSION
    CYPHAL_NODE_INFO_GIT_VERSION,
#else
    0,
#endif
    /* saturated uint8[16] unique_id */
    cyphal::support::UniqueId::instance().value(),
    /* saturated uint8[<=50] name */
    "generationmake.RobotemRovne"
  );

  /* Setup LED pins and initialize */
  pinMode(LED_2_PIN, OUTPUT);
  pinMode(LED_3_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_2_PIN, LOW);
  digitalWrite(LED_3_PIN, LOW);
  digitalWrite(LED_BUILTIN, LOW);
//  pinMode(INPUT_0_PIN, INPUT_PULLUP);
//  pinMode(INPUT_1_PIN, INPUT_PULLUP);
//  pinMode(INPUT_2_PIN, INPUT_PULLUP);
//  pinMode(INPUT_3_PIN, INPUT_PULLUP);

  /* Setup OUT0/OUT1. */
///  pinMode(OUTPUT_0_PIN, OUTPUT);
//  pinMode(OUTPUT_1_PIN, OUTPUT);
//  digitalWrite(OUTPUT_0_PIN, LOW);
//  digitalWrite(OUTPUT_1_PIN, LOW);

  /* Setup SPI access */
  SPI.begin();
  SPI.beginTransaction(MCP2515x_SPI_SETTING);
  pinMode(MCP2515_INT_PIN, INPUT_PULLUP);
  pinMode(MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MCP2515_CS_PIN, HIGH);

  /* Initialize MCP2515 */
  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_250kBPS_16MHZ);

  /* Only pass service requests/responses for this node ID through to receive buffer #0. */
  CanardFilter const CAN_FILTER_SERVICES = canardMakeFilterForServices(node_id);
  DBG_INFO("CAN Filter #1\n\r\tExt. Mask : %8X\n\r\tExt. ID   : %8X",
           CAN_FILTER_SERVICES.extended_mask,
           CAN_FILTER_SERVICES.extended_can_id);

  uint32_t const RXMB0_MASK = CAN_FILTER_SERVICES.extended_mask;
  size_t const RXMB0_FILTER_SIZE = 2;
  uint32_t const RXMB0_FILTER[RXMB0_FILTER_SIZE] =
    {
      MCP2515::CAN_EFF_BITMASK | CAN_FILTER_SERVICES.extended_can_id,
      MCP2515::CAN_EFF_BITMASK | 0
    };
  mcp2515.enableFilter(MCP2515::RxB::RxB0, RXMB0_MASK, RXMB0_FILTER, RXMB0_FILTER_SIZE);

  /* Only pass messages with subscribed port IDs. */
//  CanardFilter const CAN_FILTER_OUT_0   = canardMakeFilterForSubject(port_id_output0);
//  CanardFilter const CAN_FILTER_OUT_1   = canardMakeFilterForSubject(port_id_output1);
  CanardFilter const CAN_FILTER_LED     = canardMakeFilterForSubject(port_id_led1);
  CanardFilter const CAN_FILTER_EM_STOP = canardMakeFilterForSubject(port_id_em_stop);

  CanardFilter consolidated_filter = canardConsolidateFilters(&CAN_FILTER_LED, &CAN_FILTER_EM_STOP);
//  CanardFilter consolidated_filter = canardConsolidateFilters(&CAN_FILTERCAN_FILTER_LED_OUT_0, &CAN_FILTER_OUT_1);
//               consolidated_filter = canardConsolidateFilters(&consolidated_filter, &CAN_FILTER_LED);
//               consolidated_filter = canardConsolidateFilters(&consolidated_filter, &CAN_FILTER_EM_STOP);

  DBG_INFO("CAN Filter #2\n\r\tExt. Mask : %8X\n\r\tExt. ID   : %8X",
           consolidated_filter.extended_mask,
           consolidated_filter.extended_can_id);

  uint32_t const RXMB1_MASK = consolidated_filter.extended_mask;
  size_t const RXMB1_FILTER_SIZE = 4;
  uint32_t const RXMB1_FILTER[RXMB1_FILTER_SIZE] =
  {
    MCP2515::CAN_EFF_BITMASK | consolidated_filter.extended_can_id,
    MCP2515::CAN_EFF_BITMASK | 0,
    MCP2515::CAN_EFF_BITMASK | 0,
    MCP2515::CAN_EFF_BITMASK | 0
  };
  mcp2515.enableFilter(MCP2515::RxB::RxB1, RXMB1_MASK, RXMB1_FILTER, RXMB1_FILTER_SIZE);

  /* Leave configuration and enable MCP2515. */
  mcp2515.setNormalMode();

  // Use this initializer if using a 1.8" TFT screen:
  tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 30);
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(3);
  tft.println("Robotem Rovne");

  if (!bno.begin())
  {
    DBG_ERROR("No BNO055 detected");
    tft.setCursor(0, 90);
    tft.setTextColor(ST77XX_RED);
    tft.setTextSize(2);
    tft.println("no BNO055!");
    while (1);
  }

  /* Enable watchdog. */
  rp2040.wdt_begin(WATCHDOG_DELAY_ms);
  rp2040.wdt_reset();

  DBG_INFO("Init complete.");
}

void loop()
{
  /* Deal with all pending events of the MCP2515 -
   * signaled by the INT pin being driven LOW.
   */
  while(digitalRead(MCP2515_INT_PIN) == LOW)
    mcp2515.onExternalEventHandler();

  /* Process all pending Cyphal actions.
   */
  node_hdl.spinSome();

  /* Publish all the gathered data, although at various
   * different intervals.
   */
  static unsigned long prev_led = 0;
  static unsigned long prev_heartbeat = 0;
  static unsigned long prev_battery_voltage = 0;
  static unsigned long prev_internal_temperature = 0;
  static unsigned long prev_analog_input0 = 0;
  static unsigned long prev_analog_input1 = 0;
  static unsigned long prev_sensor = 0;
  static unsigned long prev_display = 0;

  static sensors_event_t orientationData , linearAccelData;

  unsigned long const now = millis();

  /* Publish the heartbeat once/second */
  if((now - prev_heartbeat) > UPDATE_PERIOD_HEARTBEAT_ms)
  {
    prev_heartbeat = now;

    Heartbeat_1_0 msg;

    msg.uptime = millis() / 1000;
    msg.health.value = uavcan::node::Health_1_0::NOMINAL;
    msg.mode.value = uavcan::node::Mode_1_0::OPERATIONAL;
    msg.vendor_specific_status_code = 0;

    heartbeat_pub->publish(msg);

    digitalWrite(LED_2_PIN, !digitalRead(LED_2_PIN));
  }
  if((now - prev_battery_voltage) > (update_period_ms_inputvoltage))
  {
    float const analog = analogRead(ANALOG_PIN)*3.3*11.0/1023.0;
    Serial.print("Analog Pin: ");
    Serial.println(analog);

    uavcan::primitive::scalar::Real32_1_0 uavcan_input_voltage;
    uavcan_input_voltage.value = analog;
    if(input_voltage_pub) input_voltage_pub->publish(uavcan_input_voltage);

    prev_battery_voltage = now;
  }
  if((now - prev_internal_temperature) > (update_period_ms_internaltemperature))
  {
    float const temperature = analogReadTemp();
    Serial.print("Temperature: ");
    Serial.println(temperature);

    uavcan::primitive::scalar::Real32_1_0 uavcan_internal_temperature;
    uavcan_internal_temperature.value = temperature;
    if(internal_temperature_pub) internal_temperature_pub->publish(uavcan_internal_temperature);

    prev_internal_temperature = now;
  }

  if((now - prev_analog_input0) > update_period_ms_analoginput0)
  {
    uavcan::primitive::scalar::Integer16_1_0 uavcan_analog_input0;
    uavcan_analog_input0.value = analogRead(ANALOG_INPUT_0_PIN);
    if(analog_input_0_pub) analog_input_0_pub->publish(uavcan_analog_input0);

    prev_analog_input0 = now;
  }
  if((now - prev_analog_input1) > update_period_ms_analoginput1)
  {
    uavcan::primitive::scalar::Integer16_1_0 uavcan_analog_input1;
    uavcan_analog_input1.value = analogRead(ANALOG_INPUT_1_PIN);
    if(analog_input_1_pub) analog_input_1_pub->publish(uavcan_analog_input1);

    prev_analog_input1 = now;
  }

  /* update sensor function - every 100 ms */
  if((now - prev_sensor) > 100)
  {
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    prev_sensor = now;
  }

  /* update display function - every 200 ms */
  if((now - prev_display) > 200)
  {
    tft.fillRect(0,0,20,8,ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(0);
    tft.setCursor(0, 0);
    tft.print(millis() / 1000);

    tft.setCursor(0, 140);
    if(status_em_stop==0) tft.setTextColor(ST77XX_RED);
    else tft.setTextColor(ST77XX_GREEN);
    tft.print("STOP");

    tft.fillRect(10,90,80,16,ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(10, 90);
    tft.print(orientationData.orientation.x);

    tft.fillRect(0,150,100,8,ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(0);
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    tft.setCursor(0, 150);
    tft.print(system);
    tft.setCursor(20, 150);
    tft.print(gyro);
    tft.setCursor(40, 150);
    tft.print(accel);
    tft.setCursor(60, 150);
    tft.print(mag);

    prev_display = now;
  }

  /* Feed the watchdog only if not an async reset is
   * pending because we want to restart via yakut.
   */
  if (!cyphal::support::platform::is_async_reset_pending())
    rp2040.wdt_reset();
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void onReceiveBufferFull(CanardFrame const & frame)
{
  digitalWrite(LED_3_PIN, !digitalRead(LED_3_PIN));
  node_hdl.onCanFrameReceived(frame);
}

ExecuteCommand::Response_1_1 onExecuteCommand_1_1_Request_Received(ExecuteCommand::Request_1_1 const & req)
{
  ExecuteCommand::Response_1_1 rsp;

  if (req.command == ExecuteCommand::Request_1_1::COMMAND_RESTART)
  {
    if (auto const opt_err = cyphal::support::platform::reset_async(std::chrono::milliseconds(1000)); opt_err.has_value())
    {
      DBG_ERROR("reset_async failed with error code %d", static_cast<int>(opt_err.value()));
      rsp.status = ExecuteCommand::Response_1_1::STATUS_FAILURE;
      return rsp;
    }
    rsp.status = ExecuteCommand::Response_1_1::STATUS_SUCCESS;
  }
  else if (req.command == ExecuteCommand::Request_1_1::COMMAND_STORE_PERSISTENT_STATES)
  {
    if (auto const err_mount = filesystem.mount(); err_mount.has_value())
    {
      DBG_ERROR("Mounting failed with error code %d", static_cast<int>(err_mount.value()));
      rsp.status = ExecuteCommand::Response_1_1::STATUS_FAILURE;
      return rsp;
    }
    /* Feed the watchdog. */
    rp2040.wdt_reset();
#if __GNUC__ >= 11
    auto const rc_save = cyphal::support::save(kv_storage, *node_registry, []() { rp2040.wdt_reset(); });
    if (rc_save.has_value())
    {
      DBG_ERROR("cyphal::support::save failed with %d", static_cast<int>(rc_save.value()));
      rsp.status = ExecuteCommand::Response_1_1::STATUS_FAILURE;
      return rsp;
    }
    /* Feed the watchdog. */
    rp2040.wdt_reset();
    rsp.status = ExecuteCommand::Response_1_1::STATUS_SUCCESS;
#endif /* __GNUC__ >= 11 */
    (void)filesystem.unmount();
    rsp.status = ExecuteCommand::Response_1_1::STATUS_SUCCESS;
  }
  else if (req.command == ExecuteCommand::Request_1_1::COMMAND_POWER_OFF)
  {
    /* Send the response. */
    rsp.status = ExecuteCommand::Response_1_1::STATUS_SUCCESS;

    digitalWrite(LED_2_PIN, HIGH);
    digitalWrite(LED_3_PIN, HIGH);
  }
  else if (req.command == ExecuteCommand::Request_1_1::COMMAND_BEGIN_SOFTWARE_UPDATE)
  {
    /* Send the response. */
    rsp.status = ExecuteCommand::Response_1_1::STATUS_BAD_COMMAND;
    /* not implemented yet */
  }
  else if (req.command == ExecuteCommand::Request_1_1::COMMAND_FACTORY_RESET)
  {
    /* erasing eeprom by writing FF in every cell */
    size_t const NUM_PAGES = eeprom.device_size() / eeprom.page_size();
    for(size_t page = 0; page < NUM_PAGES; page++)
    {
      uint16_t const page_addr = page * eeprom.page_size();
      eeprom.fill_page(page_addr, 0xFF);
      rp2040.wdt_reset();
    }

    /* Send the response. */
    rsp.status = ExecuteCommand::Response_1_1::STATUS_SUCCESS;
  }
  else if (req.command == ExecuteCommand::Request_1_1::COMMAND_EMERGENCY_STOP)
  {
    /* Send the response. */
    rsp.status = ExecuteCommand::Response_1_1::STATUS_BAD_COMMAND;
    /* not implemented yet */
  }
  else {
    rsp.status = ExecuteCommand::Response_1_1::STATUS_BAD_COMMAND;
  }

  return rsp;
}
