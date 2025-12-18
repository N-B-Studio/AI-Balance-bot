#include "bsp_fdcan.h"
/**
************************************************************************
* @brief:       bsp_can_init(void)
* @param:       void
* @retval:      void
* @details:     Initialize CAN peripherals and filters
************************************************************************
**/
void bsp_can_init(void)
{
	can_filter_init();
    HAL_FDCAN_Start(&hfdcan1);                               // Start FDCAN1
    HAL_FDCAN_Start(&hfdcan2);                               // Start FDCAN2
	//HAL_FDCAN_Start(&hfdcan3);
	//HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	//HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	//HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}
/**
************************************************************************
* @brief:       can_filter_init(void)
* @param:       void
* @retval:      void
* @details:     Configure CAN acceptance filters and FIFO watermarks
************************************************************************
**/
void can_filter_init(void)
{
    FDCAN_FilterTypeDef fdcan_filter;

    fdcan_filter.IdType = FDCAN_STANDARD_ID;
    fdcan_filter.FilterIndex = 0;
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1 = 0x00;
    fdcan_filter.FilterID2 = 0x00;

    // FDCAN1 filter
    HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                 FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);

    // === Added: same filter configuration for FDCAN2 ===
    HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filter);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
                                 FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 1);
}
/**
************************************************************************
* @brief:       fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan        FDCAN handle
* @param:       id            CAN identifier
* @param:       data          Pointer to transmit data
* @param:       len           Length of data in bytes
* @retval:      0 on success, non-zero on error
* @details:     Build a CAN TX header and enqueue message to TX FIFO
************************************************************************
**/
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{	
    FDCAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.Identifier   = id;
    pTxHeader.IdType       = FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType  = FDCAN_DATA_FRAME;

    // === Explicit handling of Data Length Code (DLC) based on len ===
    if (len == 0)
    {
        pTxHeader.DataLength = FDCAN_DLC_BYTES_0;
    }
    else if (len <= 8)
    {
        // Some projects use 0..8 directly as length; retain original style here
        pTxHeader.DataLength = len;
    }
    else if (len == 12)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_12;
    else if (len == 16)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_16;
    else if (len == 20)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_20;
    else if (len == 24)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_24;
    else if (len == 32)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_32;
    else if (len == 48)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_48;
    else if (len == 64)
        pTxHeader.DataLength = FDCAN_DLC_BYTES_64;
    else
    {
        // Invalid length: return error
        return 1;
    /**
    ************************************************************************
    * @brief:       fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
    * @param:       hfdcan        FDCAN handle
    * @param:       rec_id        Pointer to receive identifier output
    * @param:       buf           Buffer to store received data
    * @retval:      Number of bytes received (0 if none)
    * @details:     Read a message from FDCAN RX FIFO0 and convert DLC to byte length
    ************************************************************************
    **/
        return 1;

    return 0;
}

/**
************************************************************************
* @brief:      	fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
* @param:       hfdcan��FDCAN���
* @param:       buf���������ݻ���
* @retval:     	���յ����ݳ���
* @details:    	��������
************************************************************************
**/
uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf)
{	
	FDCAN_RxHeaderTypeDef pRxHeader;
	uint8_t len;
	
	if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &pRxHeader, buf)==HAL_OK)
	{
		*rec_id = pRxHeader.Identifier;
		if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_8)
			len = pRxHeader.DataLength;
		if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_12)
			len = 12;
		if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_16)
			len = 16;
		if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_20)
			len = 20;
		if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_24)
			len = 24;
		if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_32)
			len = 32;
		if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_48)
			len = 48;
		if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_64)
			len = 64;
		
        return len; // return received data length in bytes
	}
	return 0;	
}



uint8_t rx_data1[8] = {0};
uint16_t rec_id1;
void fdcan1_rx_callback(void)
{
	fdcanx_receive(&hfdcan1, &rec_id1, rx_data1);
}
uint8_t rx_data2[8] = {0};
uint16_t rec_id2;
void fdcan2_rx_callback(void)
{
	fdcanx_receive(&hfdcan2, &rec_id2, rx_data2);
}
//uint8_t rx_data3[8] = {0};
//uint16_t rec_id3;
//void fdcan3_rx_callback(void)
//{
	//fdcanx_receive(&hfdcan3, &rec_id3, rx_data3);
//}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(hfdcan == &hfdcan1)
	{
		fdcan1_rx_callback();
	}
	if(hfdcan == &hfdcan2)
	{
		fdcan2_rx_callback();
	}
	//if(hfdcan == &hfdcan3)
	//{
	//	fdcan3_rx_callback();
	//}
}











