/**
 * @file       TinyGsmClientSIM7020.h
 * @author     Volodymyr Shymanskyy
 * @license    LGPL-3.0
 * @copyright  Copyright (c) 2016 Volodymyr Shymanskyy
 * @date       Nov 2016
 */

#ifndef SRC_TINYGSMCLIENTSIM7020_H_
#define SRC_TINYGSMCLIENTSIM7020_H_
// #pragma message("TinyGSM:  TinyGsmClientSIM7020")

// #define TINY_GSM_DEBUG Serial
// #define TINY_GSM_USE_HEX

#define TINY_GSM_YIELD_MS 3

#define TINY_GSM_MUX_COUNT 5
#define TINY_GSM_BUFFER_READ_AND_CHECK_SIZE

#include "TinyGsmNBIOT.tpp"
#include "TinyGsmModem.tpp"
#include "TinyGsmTCP.tpp"
#include "TinyGsmTime.tpp"

#define GSM_NL "\r\n"
static const char GSM_OK[] TINY_GSM_PROGMEM    = "OK" GSM_NL;
static const char GSM_ERROR[] TINY_GSM_PROGMEM = "ERROR" GSM_NL;
#if defined       TINY_GSM_DEBUG
static const char GSM_CME_ERROR[] TINY_GSM_PROGMEM = GSM_NL "+CME ERROR:";
static const char GSM_CMS_ERROR[] TINY_GSM_PROGMEM = GSM_NL "+CMS ERROR:";
#endif

enum RegStatus {
  REG_NO_RESULT    = -1,
  REG_UNREGISTERED = 0,
  REG_SEARCHING    = 2,
  REG_DENIED       = 3,
  REG_OK_HOME      = 1,
  REG_OK_ROAMING   = 5,
  REG_UNKNOWN      = 4,
};
class TinyGsmSim7020 : public TinyGsmModem<TinyGsmSim7020>,
                       public TinyGsmNBIOT<TinyGsmSim7020>,
                       public TinyGsmTCP<TinyGsmSim7020, TINY_GSM_MUX_COUNT>,
                       public TinyGsmTime<TinyGsmSim7020> {
  friend class TinyGsmModem<TinyGsmSim7020>;
  friend class TinyGsmNBIOT<TinyGsmSim7020>;
  friend class TinyGsmTCP<TinyGsmSim7020, TINY_GSM_MUX_COUNT>;
  friend class TinyGsmTime<TinyGsmSim7020>;

  /*
   * Inner Client
   */
 public:
  class GsmClientSim7020 : public GsmClient {
    friend class TinyGsmSim7020;

   public:
    GsmClientSim7020() {}

    explicit GsmClientSim7020(TinyGsmSim7020& modem, uint8_t mux = 0) {
      init(&modem, mux);
    }

    bool init(TinyGsmSim7020* modem, uint8_t mux = 0) {
      this->at       = modem;
      sock_available = 0;
      prev_check     = 0;
      sock_connected = false;
      got_data       = false;

      if (mux < TINY_GSM_MUX_COUNT) {
        this->mux = mux;
      } else {
        this->mux = (mux % TINY_GSM_MUX_COUNT);
      }
      at->sockets[this->mux] = this;

      return true;
    }

   public:
    virtual int connect(const char* host, uint16_t port, int timeout_s) {
      stop();
      TINY_GSM_YIELD();
      rx.clear();
      sock_connected = at->modemConnect(host, port, mux, timeout_s);
      return sock_connected;
    }
    TINY_GSM_CLIENT_CONNECT_OVERRIDES

    void stop(uint32_t maxWaitMs) {
      dumpModemBuffer(maxWaitMs);
      at->sendAT(GF("+CSOCL="), mux);
      sock_connected = false;
      at->waitResponse();
    }
    void stop() override {
      stop(15000L);
    }

    /*
     * Extended API
     */

    String remoteIP() TINY_GSM_ATTR_NOT_IMPLEMENTED;
  };

  /*
   * Inner Secure Client
   */
  // TODO: SSL Client
  /*
   * Constructor
   */
 public:
  explicit TinyGsmSim7020(Stream& stream, uint8_t reset_pin)
      : stream(stream),
        reset_pin(reset_pin) {
    memset(sockets, 0, sizeof(sockets));
  }

  /*
   * Basic functions
   */
 protected:
  bool initImpl(const char* pin = NULL) {
    restart();

    DBG(GF("### TinyGSM Version:"), TINYGSM_VERSION);
    DBG(GF("### TinyGSM Compiled Module:  TinyGsmClientSIM7020"));

    if (!testAT()) { return false; }

    sendAT(GF("&FZ"));  // Factory + Reset
    waitResponse();

    sendAT(GF("E0"));  // Echo Off
    if (waitResponse() != 1) { return false; }

#ifdef TINY_GSM_DEBUG
    sendAT(GF("+CMEE=2"));  // turn on verbose error codes
#else
    sendAT(GF("+CMEE=0"));  // turn off error codes
#endif
    waitResponse();

    DBG(GF("### Modem:"), getModemName());

    // Enable Local Time Stamp for getting network time
    sendAT(GF("+CLTS=1"));
    if (waitResponse(10000L) != 1) { return false; }

    // Enable battery checks
    sendAT(GF("+CBATCHK=1"));
    waitResponse();

    SimStatus ret = getSimStatus();
    // if the sim isn't ready and a pin has been provided, try to unlock the sim
    if (ret != SIM_READY && pin != NULL && strlen(pin) > 0) {
      simUnlock(pin);
      return (getSimStatus() == SIM_READY);
    } else {
      // if the sim is ready, or it's locked but no pin has been provided,
      // return true
      return (ret == SIM_READY || ret == SIM_LOCKED);
    }
  }

  String getModemNameImpl() {
    sendAT(GF("+CGMI"));
    String res1;
    if (waitResponse(1000L, res1) != 1) { return "unknown"; }
    res1.replace("\r\nOK\r\n", "");
    res1.replace("\rOK\r", "");
    res1.trim();

    sendAT(GF("+GMM"));
    String res2;
    if (waitResponse(1000L, res2) != 1) { return "unknown"; }
    res2.replace("\r\nOK\r\n", "");
    res2.replace("\rOK\r", "");
    res2.trim();

    String name = res1 + String(' ') + res2;
    DBG("### Modem:", name);
    return name;
  }

  bool factoryDefaultImpl() {
    sendAT(GF("&F0"));  // Factory
    waitResponse();
    sendAT(GF("Z0"));  // Reset
    waitResponse();
    sendAT(GF("E0"));  // Echo Off
    waitResponse();
    sendAT(GF("&W"));  // Write
    waitResponse();
    sendAT(GF("+CLTS=0"));  // Disable Get Local Timestamp
    waitResponse();
    sendAT(GF("+CSOSENDFLAG=0"));  // Disable TCP Send Flag
    waitResponse();
    sendAT(GF("+IPR=0"));  // Auto-baud
    waitResponse();
    sendAT(GF("+IFC=0,0"));  // No Flow Control
    waitResponse();
    sendAT(GF("+ICF=3,3"));  // 8 data 0 parity 1 stop
    waitResponse();
    sendAT(GF("+CSCLK=0"));  // Disable Slow Clock
    waitResponse();
    sendAT(GF("&W"));  // Write configuration
    return waitResponse() == 1;
  }

  /*
   * Power functions
   */
 protected:
  bool restartImpl() {
    /* Hardware Reset */
    pinMode(this->reset_pin, OUTPUT);
    digitalWrite(this->reset_pin, LOW);
    delay(300);
    digitalWrite(this->reset_pin, HIGH);
    delay(5000);

    return true;
  }

  bool powerOffImpl() {
    sendAT(GF("+CPOWD=1"));
    return waitResponse(10000L, GF("NORMAL POWER DOWN")) == 1;
  }

  // During sleep, the SIM7020 module has its serial communication disabled. In
  // order to reestablish communication pull the DRT-pin of the SIM7020 module
  // LOW for at least 50ms. Then use this function to disable sleep mode. The
  // DTR-pin can then be released again.
  bool sleepEnableImpl(bool enable = true) {
    sendAT(GF("+CSCLK="), enable);
    return waitResponse() == 1;
  }

  /*
   * Generic network functions
   */
 public:
  RegStatus getRegistrationStatus() {
    return (RegStatus)getRegistrationStatusXREG("CEREG");
  }

 protected:
  bool isNetworkConnectedImpl() {
    RegStatus s = getRegistrationStatus();
    return (s == REG_OK_HOME || s == REG_OK_ROAMING);
  }

  String getLocalIPImpl() {
    sendAT(GF("+CGPADDR=1"));
    if (waitResponse(GF("+CGPADDR:")) != 1) { return ""; }
    streamSkipUntil('\"');  // Skip context id
    String res = stream.readStringUntil('\"');
    if (waitResponse() != 1) { return ""; }
    return res;
  }

  /*
   * GPRS functions
   */
 protected:
  // No functions of this type supported

  /*
   * NBIOT functions
   */
 protected:
  bool nbiotConnectImpl(const char* apn, uint8_t band = 0) {
    // Set APN
    sendAT("*MCGDEFCONT=", GF("\"IP\",\""), apn, GF("\""));
    if (waitResponse() != 1) { return false; }
    // Set Band
    sendAT("+CBAND=", band);
    if (waitResponse() != 1) { return false; }
    return true;
  }

  /*
   * SIM card functions
   */
 protected:
  // May not return the "+CCID" before the number
  String getSimCCIDImpl() {
    sendAT(GF("+CCID"));
    if (waitResponse(GF(GSM_NL)) != 1) { return ""; }
    String res = stream.readStringUntil('\n');
    waitResponse();
    // Trim out the CCID header in case it is there
    res.replace("CCID:", "");
    res.trim();
    return res;
  }

  /*
   * Phone Call functions
   */
 public:
  /*
   * Messaging functions
   */
 protected:
  // Follows all messaging functions per template

  /*
   * GSM Location functions
   */
 protected:
  /*
   * GPS/GNSS/GLONASS location functions
   */
 protected:
  // No functions of this type supported

  /*
   * Time functions
   */
 protected:
  // Can follow the standard CCLK function in the template

  /*
   * Battery functions
   */
 protected:
  // Follows all battery functions per template

  /*
   * NTP server functions
   */
 public:
  boolean isValidNumber(String str) {
    if (!(str.charAt(0) == '+' || str.charAt(0) == '-' ||
          isDigit(str.charAt(0))))
      return false;

    for (byte i = 1; i < str.length(); i++) {
      if (!(isDigit(str.charAt(i)) || str.charAt(i) == '.')) { return false; }
    }
    return true;
  }

  bool NTPServerSync(String server = "pool.ntp.org", byte TimeZone = 32) {
    // AT+CURTC Control CCLK Show UTC Or RTC Time
    // Use AT CCLK? command to get UTC Or RTC Time
    // Start to query network time
    sendAT(GF("+CSNTPSTART="), '\"', server, GF("\",\"+"), String(TimeZone),
           '\"');
    if (waitResponse(10000L) != 1) { return false; }

    // Stop to query network time
    sendAT(GF("+CSNTPSTOP"));
    if (waitResponse(10000L) != 1) { return false; }
    return true;
  }

  /*
   * Client related functions
   */
 protected:
  bool modemConnect(const char* host, uint16_t port, uint8_t mux,
                    int timeout_s = 75) {
    bool     rsp = true;
    String   ip_addr;
    uint32_t timeout_ms = ((uint32_t)timeout_s) * 1000;
    /* Query the IP Address of Given Domain Name */
    sendAT(GF("+CDNSGIP="), host);
    if (waitResponse(10000L, GF("+CDNSGIP:"))) {
      if (waitResponse(5000L, ip_addr, GF(GSM_NL)) != 1) { return 0; }
      int n1  = ip_addr.lastIndexOf(',');
      int n2  = ip_addr.lastIndexOf('\"');
      ip_addr = ip_addr.substring(n1 + 2, n2);
    }
    /* Enable TCP Send Flag */
    sendAT(GF("+CSOSENDFLAG=1"));
    if (waitResponse() != 1) { return 0; }
    /* Enable getting data from network manually */
    sendAT(GF("+CSORXGET=1"));
    if (waitResponse() != 1) { return 0; }
    /* Creat a TCP/UDP Socket */
    sendAT(GF("+CSOC=1,1,1"));
    rsp &= waitResponse();
    /* Connect Socket to Remote Address and Port */
    sendAT(GF("+CSOCON="), mux, ',', port, ',', GF("\""), ip_addr.c_str(),
           GF("\""));
    rsp &= waitResponse(timeout_ms);
    return rsp;
  }

  int16_t modemSend(const void* buff, size_t len, uint8_t mux) {
    /* Send Data to Remote Via Socket With Data Mode */
    sendAT(GF("+CSODSEND="), mux, ',', (uint16_t)len);
    if (waitResponse(GF(">")) != 1) { return 0; }
    stream.write(reinterpret_cast<const uint8_t*>(buff), len);
    stream.flush();
    /* Note: If the upper layer uses SSL, waitResponse timeout_ms will take a
     * long time. The default is 15s. */
    if (waitResponse(15000L, GF(GSM_NL "SEND:")) != 1) { return 0; }
    streamSkipUntil(',');  // Skip mux
    return streamGetIntBefore('\n');
  }

  size_t modemRead(size_t size, uint8_t mux) {
    if (!sockets[mux]) return 0;
      /* Get Data from Network Manually */
#ifdef TINY_GSM_USE_HEX
    sendAT(GF("+CSORXGET=3,"), mux, ',', (uint16_t)size);
    if (waitResponse(GF("+CSORXGET:")) != 1) { return 0; }
#else
    sendAT(GF("+CSORXGET=2,"), mux, ',', (uint16_t)size);
    if (waitResponse(GF("+CSORXGET:")) != 1) { return 0; }
#endif
    streamSkipUntil(',');  // Skip Rx mode 2/normal or 3/HEX
    streamSkipUntil(',');  // Skip mux
    int16_t len_requested = streamGetIntBefore(',');
    //  ^^ Requested number of data bytes (1-1460 bytes)to be read
    int16_t len_confirmed = streamGetIntBefore('\n');
    // ^^ Confirmed number of data bytes to be read, which may be less than
    // requested. 0 indicates that no data can be read.
    // SRGD NOTE:  Contrary to above (which is copied from AT command manual)
    // this is actually be the number of bytes that will be remaining in the
    // buffer after the read.
    for (int i = 0; i < len_requested; i++) {
      uint32_t startMillis = millis();
#ifdef TINY_GSM_USE_HEX
      while (stream.available() < 2 &&
             (millis() - startMillis < sockets[mux]->_timeout)) {
        TINY_GSM_YIELD();
      }
      char buf[4] = {
          0,
      };
      buf[0] = stream.read();
      buf[1] = stream.read();
      char c = strtol(buf, NULL, 16);
#else
      while (!stream.available() &&
             (millis() - startMillis < sockets[mux]->_timeout)) {
        TINY_GSM_YIELD();
      }
      char c = stream.read();
#endif
      sockets[mux]->rx.put(c);
    }
    // DBG("### READ:", len_requested, "from", mux);
    // sockets[mux]->sock_available = modemGetAvailable(mux);
    sockets[mux]->sock_available = len_confirmed;
    waitResponse();
    return len_requested;
  }

  size_t modemGetAvailable(uint8_t mux) {
    if (!sockets[mux]) return 0;
    sendAT(GF("+CSORXGET=4,"), mux);
    size_t result = 0;
    if (waitResponse(GF("+CSORXGET:")) == 1) {
      streamSkipUntil(',');  // Skip mode 4
      streamSkipUntil(',');  // Skip mux
      result = streamGetIntBefore('\n');
      waitResponse();
    }
    // DBG("### Available:", result, "on", mux);
    if (!result) { sockets[mux]->sock_connected = modemGetConnected(mux); }
    return result;
  }

  bool modemGetConnected(uint8_t mux) {
    /* Get Socket Status */
    sendAT(GF("+CSOSTATUS="), mux);
    waitResponse(GF("+CSOSTATUS:"));
    streamSkipUntil(',');  // Skip mux
    // int8_t res = waitResponse(GF("0"), GF("1"), GF("2"));
    int8_t res = (int8_t)streamGetIntBefore('\n');
    waitResponse();
    return (res == 2);
  }

  /*
   * Utilities
   */
 public:
  int8_t waitResponse(uint32_t timeout_ms, String& data,
                      GsmConstStr r1 = GFP(GSM_OK),
                      GsmConstStr r2 = GFP(GSM_ERROR),
#if defined TINY_GSM_DEBUG
                      GsmConstStr r3 = GFP(GSM_CME_ERROR),
                      GsmConstStr r4 = GFP(GSM_CMS_ERROR),
#else
                      GsmConstStr r3 = NULL, GsmConstStr r4 = NULL,
#endif
                      GsmConstStr r5 = NULL) {
    data.reserve(64);
    uint8_t  index       = 0;
    uint32_t startMillis = millis();
    do {
      TINY_GSM_YIELD();
      while (stream.available() > 0) {
        TINY_GSM_YIELD();
        int8_t a = stream.read();
        if (a <= 0) continue;  // Skip 0x00 bytes, just in case
        data += static_cast<char>(a);
        if (r1 && data.endsWith(r1)) {
          index = 1;
          goto finish;
        } else if (r2 && data.endsWith(r2)) {
          index = 2;
          goto finish;
        } else if (r3 && data.endsWith(r3)) {
#if defined TINY_GSM_DEBUG
          if (r3 == GFP(GSM_CME_ERROR)) {
            streamSkipUntil('\n');  // Read out the error
          }
#endif
          index = 3;
          goto finish;
        } else if (r4 && data.endsWith(r4)) {
          index = 4;
          goto finish;
        } else if (r5 && data.endsWith(r5)) {
          index = 5;
          goto finish;
        } else if (data.endsWith(GF(GSM_NL "+CSORXGET:"))) {
          int8_t mode = streamGetIntBefore(',');
          if (mode == 1) {
            int8_t mux = streamGetIntBefore('\n');
            if (mux >= 0 && mux < TINY_GSM_MUX_COUNT && sockets[mux]) {
              sockets[mux]->got_data = true;
            }
            data = "";
            // DBG("### Got Data:", mux);
          } else {
            data += mode;
          }
        } else if (data.endsWith(GF("+CSOERR:"))) {
          int8_t mux = streamGetIntBefore(',');
          if (mux >= 0 && mux < TINY_GSM_MUX_COUNT && sockets[mux]) {
            sockets[mux]->sock_connected = false;
          }
          data = "";
          // DBG("### Closed: ", mux);
        } else if (data.endsWith(GF("+CLTS"))) {
          streamSkipUntil('\n');
          data = "";
          DBG("### got Network time.");
        }
      }
    } while (millis() - startMillis < timeout_ms);
  finish:
    if (!index) {
      data.trim();
      if (data.length()) { DBG("### Unhandled:", data); }
      data = "";
    }
    // data.replace(GSM_NL, "/");
    // DBG('<', index, '>', data);
    return index;
  }

  int8_t waitResponse(uint32_t timeout_ms, GsmConstStr r1 = GFP(GSM_OK),
                      GsmConstStr r2 = GFP(GSM_ERROR),
#if defined TINY_GSM_DEBUG
                      GsmConstStr r3 = GFP(GSM_CME_ERROR),
                      GsmConstStr r4 = GFP(GSM_CMS_ERROR),
#else
                      GsmConstStr r3 = NULL, GsmConstStr r4 = NULL,
#endif
                      GsmConstStr r5 = NULL) {
    String data;
    return waitResponse(timeout_ms, data, r1, r2, r3, r4, r5);
  }

  int8_t waitResponse(GsmConstStr r1 = GFP(GSM_OK),
                      GsmConstStr r2 = GFP(GSM_ERROR),
#if defined TINY_GSM_DEBUG
                      GsmConstStr r3 = GFP(GSM_CME_ERROR),
                      GsmConstStr r4 = GFP(GSM_CMS_ERROR),
#else
                      GsmConstStr r3 = NULL, GsmConstStr r4 = NULL,
#endif
                      GsmConstStr r5 = NULL) {
    return waitResponse(1000, r1, r2, r3, r4, r5);
  }

 public:
  Stream&       stream;
  uint8_t       reset_pin;
  unsigned long baud;

 protected:
  GsmClientSim7020* sockets[TINY_GSM_MUX_COUNT];
  const char*       gsmNL = GSM_NL;
};

#endif  // SRC_TINYGSMCLIENTSIM7020_H_
