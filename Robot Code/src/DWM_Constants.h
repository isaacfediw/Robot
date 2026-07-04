// DWM Constants Header
// Authour -> Isaac Fediw
// Date Created -> 2026-06-17

// ***** Headers ***** //
// Headers do NOT require shifting
#define SHORT_READ   0x00
#define SHORT_WRITE  0x80
#define FAST_CMD     0x81
#define FULL_READ    0x40
#define FULL_WRITE   0xC0
#define MASKED_WRITE 0xC0
// ******************** //

// ***** Main Register Addresses ***** //
#define MAIN_REGISTER 0x00
#define DRX_REGISTER  0x06 // digital receiver tuning and configuration
#define RF_CAL        0x08 // transmitter calibration
#define ALWAYS_ON     0x0A // always on system control block
#define OTP_REGISTER  0x0B // one time programmable interface
#define PMSC_CTRL     0x11 // power management, timing, and sequence control
#define RX_BUFFER0    0x12
#define TX_BUFFER     0x14
// *********************************** //

// ***** Sub Register Addresses ***** //
// main register sub addresses
#define DEV_ID      0x00
#define EUI_64      0x04
#define SYS_CFG     0x10
#define TX_FCTRL    0x24
#define SYS_STATUS  0x44
#define RX_FINFO    0x4C
#define RX_TIME     0x64

// always-on register sub addresses
#define AON_DIG_CGF 0x00
#define AON_CTRL    0x04

// otp register sub addresses
#define OTP_CFG     0x08

// pmsc register sub addresses
#define CLK_CTRL    0x04

// ********************************** //

// ***** Bit Masks ***** //
#define CPLOCK 0x02 // Clock PLL Lock Status, from SYS_STATUS sub register
// ********************* //

// ***** Fast Commands ****** //
#define CMD_TRXOFF      0x00  // Force IDLE, clear events
#define CMD_TX          0x01  // Start TX immediately
#define CMD_RX          0x02  // Enable RX immediately

#define CMD_DTX         0x03  // Delayed TX (DX_TIME)
#define CMD_DRX         0x04  // Delayed RX (DX_TIME)

#define CMD_DTX_TS      0x05  // Delayed TX (TX timestamp + DX_TIME)
#define CMD_DRX_TS      0x06  // Delayed RX (TX timestamp + DX_TIME)

#define CMD_DTX_RS      0x07  // Delayed TX (RX timestamp + DX_TIME)
#define CMD_DRX_RS      0x08  // Delayed RX (RX timestamp + DX_TIME)

#define CMD_DTX_REF     0x09  // Delayed TX (DREF_TIME + DX_TIME)
#define CMD_DRX_REF     0x0A  // Delayed RX (DREF_TIME + DX_TIME)

#define CMD_CCA_TX      0x0B  // TX if channel is clear
#define CMD_TX_W4R      0x0C  // TX then enable RX

#define CMD_DTX_W4R     0x0D  // Delayed TX then RX
#define CMD_DTX_TS_W4R  0x0E  // Delayed TX (TX time ref) then RX
#define CMD_DTX_RS_W4R  0x0F  // Delayed TX (RX time ref) then RX
#define CMD_DTX_REF_W4R 0x10  // Delayed TX (DREF_TIME ref) then RX

#define CMD_CCA_TX_W4R  0x11  // CCA TX then RX

#define CMD_CLR_IRQS    0x12  // Clear all IRQ events

#define CMD_DB_TOGGLE   0x13  // Toggle RX double buffer
// ************************** //