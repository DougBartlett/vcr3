#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// $[CMU]
// [CMU]$

// $[LFXO]
// LFXO LFXTAL_I on PD01
#ifndef LFXO_LFXTAL_I_PORT                      
#define LFXO_LFXTAL_I_PORT                       gpioPortD
#endif
#ifndef LFXO_LFXTAL_I_PIN                       
#define LFXO_LFXTAL_I_PIN                        1
#endif

// LFXO LFXTAL_O on PD00
#ifndef LFXO_LFXTAL_O_PORT                      
#define LFXO_LFXTAL_O_PORT                       gpioPortD
#endif
#ifndef LFXO_LFXTAL_O_PIN                       
#define LFXO_LFXTAL_O_PIN                        0
#endif

// [LFXO]$

// $[PRS.ASYNCH0]
// [PRS.ASYNCH0]$

// $[PRS.ASYNCH1]
// [PRS.ASYNCH1]$

// $[PRS.ASYNCH2]
// [PRS.ASYNCH2]$

// $[PRS.ASYNCH3]
// [PRS.ASYNCH3]$

// $[PRS.ASYNCH4]
// [PRS.ASYNCH4]$

// $[PRS.ASYNCH5]
// [PRS.ASYNCH5]$

// $[PRS.ASYNCH6]
// [PRS.ASYNCH6]$

// $[PRS.ASYNCH7]
// [PRS.ASYNCH7]$

// $[PRS.ASYNCH8]
// [PRS.ASYNCH8]$

// $[PRS.ASYNCH9]
// [PRS.ASYNCH9]$

// $[PRS.ASYNCH10]
// [PRS.ASYNCH10]$

// $[PRS.ASYNCH11]
// [PRS.ASYNCH11]$

// $[PRS.SYNCH0]
// [PRS.SYNCH0]$

// $[PRS.SYNCH1]
// [PRS.SYNCH1]$

// $[PRS.SYNCH2]
// [PRS.SYNCH2]$

// $[PRS.SYNCH3]
// [PRS.SYNCH3]$

// $[GPIO]
// GPIO SWV on PA03
#ifndef GPIO_SWV_PORT                           
#define GPIO_SWV_PORT                            gpioPortA
#endif
#ifndef GPIO_SWV_PIN                            
#define GPIO_SWV_PIN                             3
#endif

// [GPIO]$

// $[TIMER0]
// [TIMER0]$

// $[TIMER1]
// [TIMER1]$

// $[TIMER2]
// [TIMER2]$

// $[TIMER3]
// [TIMER3]$

// $[TIMER4]
// [TIMER4]$

// $[USART0]
// [USART0]$

// $[USART1]
// [USART1]$

// $[I2C1]
// I2C1 SCL on PD02
#ifndef I2C1_SCL_PORT                           
#define I2C1_SCL_PORT                            gpioPortD
#endif
#ifndef I2C1_SCL_PIN                            
#define I2C1_SCL_PIN                             2
#endif

// I2C1 SDA on PD03
#ifndef I2C1_SDA_PORT                           
#define I2C1_SDA_PORT                            gpioPortD
#endif
#ifndef I2C1_SDA_PIN                            
#define I2C1_SDA_PIN                             3
#endif

// [I2C1]$

// $[PDM]
// [PDM]$

// $[LETIMER0]
// [LETIMER0]$

// $[IADC0]
// [IADC0]$

// $[I2C0]
// [I2C0]$

// $[EUART0]
// EUART0 CTS on PA08
#ifndef EUART0_CTS_PORT                         
#define EUART0_CTS_PORT                          gpioPortA
#endif
#ifndef EUART0_CTS_PIN                          
#define EUART0_CTS_PIN                           8
#endif

// EUART0 RTS on PA07
#ifndef EUART0_RTS_PORT                         
#define EUART0_RTS_PORT                          gpioPortA
#endif
#ifndef EUART0_RTS_PIN                          
#define EUART0_RTS_PIN                           7
#endif

// EUART0 RX on PA06
#ifndef EUART0_RX_PORT                          
#define EUART0_RX_PORT                           gpioPortA
#endif
#ifndef EUART0_RX_PIN                           
#define EUART0_RX_PIN                            6
#endif

// EUART0 TX on PA05
#ifndef EUART0_TX_PORT                          
#define EUART0_TX_PORT                           gpioPortA
#endif
#ifndef EUART0_TX_PIN                           
#define EUART0_TX_PIN                            5
#endif

// [EUART0]$

// $[PTI]
// PTI DFRAME on PC05
#ifndef PTI_DFRAME_PORT                         
#define PTI_DFRAME_PORT                          gpioPortC
#endif
#ifndef PTI_DFRAME_PIN                          
#define PTI_DFRAME_PIN                           5
#endif

// PTI DOUT on PC04
#ifndef PTI_DOUT_PORT                           
#define PTI_DOUT_PORT                            gpioPortC
#endif
#ifndef PTI_DOUT_PIN                            
#define PTI_DOUT_PIN                             4
#endif

// [PTI]$

// $[MODEM]
// [MODEM]$

// $[CUSTOM_PIN_NAME]
#ifndef _PORT                                   
#define _PORT                                    gpioPortA
#endif
#ifndef _PIN                                    
#define _PIN                                     0
#endif

// [CUSTOM_PIN_NAME]$

#endif // PIN_CONFIG_H

