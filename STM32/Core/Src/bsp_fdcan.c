#include "bsp_fdcan.h"
/**
************************************************************************
* @brief:      	bsp_can_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN ʹ��
************************************************************************
**/
void bsp_can_init(void)
{
	can_filter_init();
	HAL_FDCAN_Start(&hfdcan1);                               //����FDCAN
	HAL_FDCAN_Start(&hfdcan2);
	//HAL_FDCAN_Start(&hfdcan3);
	//HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	//HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	//HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}
/**
************************************************************************
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN�˲�����ʼ��
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

    // FDCAN1 过滤器
    HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                 FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);

    // === 新增：FDCAN2 同样的过滤器 ===
    HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filter);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
                                 FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 1);
}
/**
************************************************************************
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcanFDCAN
* @param:       id��CAN�豸ID
* @param:       data
* @param:       len
* @retval:     	void
* @details:    	��������
************************************************************************
**/
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{	
    FDCAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.Identifier   = id;
    pTxHeader.IdType       = FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType  = FDCAN_DATA_FRAME;

    // === 显式处理 len ===
    if (len == 0)
    {
        pTxHeader.DataLength = FDCAN_DLC_BYTES_0;
    }
    else if (len <= 8)
    {
        // 有些工程里直接用 0..8 作为长度，这里保持你原来的风格
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
        // 非法长度，直接返回错误
        return 1;
    }

    pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch       = FDCAN_BRS_ON;
    pTxHeader.FDFormat            = FDCAN_FD_CAN;
    pTxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker       = 0;
 
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data) != HAL_OK)
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
		
		return len;//��������
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











