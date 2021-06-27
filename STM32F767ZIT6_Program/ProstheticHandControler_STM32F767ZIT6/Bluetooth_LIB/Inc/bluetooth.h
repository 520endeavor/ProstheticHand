#ifndef _BLUETOOTH_H_
#define _BLUETOOTH_H_

typedef unsigned          char uint8_t;
typedef unsigned short    int  uint16_t;
typedef   signed          char int8_t;

#define recv_buff_index_max 500       //number of receive bytes 
#define EmgChannel 8                  //number of channel
#define DataBufferSize 800            //deepth of circle buffer


extern uint8_t recv_buff[recv_buff_index_max]; //DMA receive Buffer
extern double DataBuffer[DataBufferSize][EmgChannel]; //circle buffer

/***********************************************************************************
 * initiate_filter                                                                 *
 * .Description:                                                                   *
 *     To initiate the filter                                                      *
 * .Parameters                                                                     *
 *     None                                                                        *
 * .Return value:                                                                  *
 *     None                                                                        *
 * Attention                                                                       *
 *     Add this function at the beginning of main function                         *
 ***********************************************************************************/
void initiate_filter(void);

/***********************************************************************************
 * deal_with_recv_buff                                                             *
 * .Description:                                                                   *
 *     To decode the data from data_in buffer, and store the data processed into   *
 *     data_out buffer                                                             *
 * .Parameters                                                                     *
 *     1. data_in          - the buffer to contained the received Data             *
 * .Return value:                                                                  *
 *     - Index of DataBuffer                                                       *
 ***********************************************************************************/
uint16_t deal_with_recv_buff(uint8_t *data_in);

#endif
