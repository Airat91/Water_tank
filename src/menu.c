#include "menu.h"
#include "main.h"
#include "buttons.h"
#include "dcts.h"
#include "dcts_config.h"
#include "string.h"

/**
  * @defgroup menu
  * @brief work with menu
  */


#define NULL_ENTRY Null_Menu
static menuItem Null_Menu = {
    .Next = (void*)0,
    .Previous = (void*)0,
    .Parent = (void*)0,
    .Child = (void*)0,
    .Child_num = 0,
    .Page = 0,
    .Text = {0},
};

menuItem* selectedMenuItem;
static menuItem* menuStack[10];
static volatile uint8_t menuStackTop;

//                  NAME           NEXT            PREV            PARENT          CHILD        GHILD_NUM   PAGE                    TEXT
MAKE_MENU       (main_page,     NULL_ENTRY,     NULL_ENTRY,     NULL_ENTRY,     common_info,    4,          MAIN_PAGE,          "������� ����");
//MAKE_MENU       (main_menu,     main_page,      main_page,      NULL_ENTRY,     common_info,    0,          MAIN_MENU,          "������� ����");
  MAKE_MENU     (common_info,   meas_channels,  tmpr_calib,     main_page,      info,           0,          COMMON_INFO,        "�� ����������");
    MAKE_MENU   (info,          NULL_ENTRY,     NULL_ENTRY,     common_info,    NULL_ENTRY,     0,          INFO,               "�� ����������");
  MAKE_MENU     (meas_channels, lvl_calib,      common_info,    main_page,      meas_ch_0,      13,         MEAS_CHANNELS,      "���. ������");
    MAKE_MENU   (meas_ch_0,     meas_ch_1,      meas_ch_12,     meas_channels,  NULL_ENTRY,     0,          MEAS_CH_0,          0x00);
    MAKE_MENU   (meas_ch_1,     meas_ch_2,      meas_ch_0,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_1,          0x00);
    MAKE_MENU   (meas_ch_2,     meas_ch_3,      meas_ch_1,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_2,          0x00);
    MAKE_MENU   (meas_ch_3,     meas_ch_4,      meas_ch_2,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_3,          0x00);
    MAKE_MENU   (meas_ch_4,     meas_ch_5,      meas_ch_3,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_4,          0x00);
    MAKE_MENU   (meas_ch_5,     meas_ch_6,      meas_ch_4,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_5,          0x00);
    MAKE_MENU   (meas_ch_6,     meas_ch_7,      meas_ch_5,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_6,          0x00);
    MAKE_MENU   (meas_ch_7,     meas_ch_8,      meas_ch_6,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_7,          0x00);
    MAKE_MENU   (meas_ch_8,     meas_ch_9,      meas_ch_7,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_8,          0x00);
    MAKE_MENU   (meas_ch_9,     meas_ch_10,     meas_ch_8,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_9,          0x00);
    MAKE_MENU   (meas_ch_10,    meas_ch_11,     meas_ch_9,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_10,         0x00);
    MAKE_MENU   (meas_ch_11,    meas_ch_12,     meas_ch_10,     meas_channels,  NULL_ENTRY,     0,          MEAS_CH_11,         0x00);
    MAKE_MENU   (meas_ch_12,    meas_ch_0,      meas_ch_11,     meas_channels,  NULL_ENTRY,     0,          MEAS_CH_12,         0x00);
  MAKE_MENU     (lvl_calib,     tmpr_calib,     meas_channels,  main_page,      calib_a,        2,          LVL_CALIB,          "�����. ������");
    MAKE_MENU   (calib_a,       calib_b,        calib_b,        lvl_calib,      NULL_ENTRY,     0,          LVL_CALIB_A,        "����. �");
    MAKE_MENU   (calib_b,       calib_a,        calib_a,        lvl_calib,      NULL_ENTRY,     0,          LVL_CALIB_B,        "����. �");
  MAKE_MENU     (tmpr_calib,    common_info,    lvl_calib,      main_page,      adc_0,          11,         TMPR_CALIB,         "�����. ������.");
    MAKE_MENU   (adc_0,         adc_10,         adc_100,        tmpr_calib,     NULL_ENTRY,     0,          ADC_0,              "��� ��� 0�C");
    MAKE_MENU   (adc_10,        adc_20,         adc_0,          tmpr_calib,     NULL_ENTRY,     0,          ADC_10,             "��� ��� 10�C");
    MAKE_MENU   (adc_20,        adc_30,         adc_10,         tmpr_calib,     NULL_ENTRY,     0,          ADC_20,             "��� ��� 20�C");
    MAKE_MENU   (adc_30,        adc_40,         adc_20,         tmpr_calib,     NULL_ENTRY,     0,          ADC_30,             "��� ��� 30�C");
    MAKE_MENU   (adc_40,        adc_50,         adc_30,         tmpr_calib,     NULL_ENTRY,     0,          ADC_40,             "��� ��� 40�C");
    MAKE_MENU   (adc_50,        adc_60,         adc_40,         tmpr_calib,     NULL_ENTRY,     0,          ADC_50,             "��� ��� 50�C");
    MAKE_MENU   (adc_60,        adc_70,         adc_50,         tmpr_calib,     NULL_ENTRY,     0,          ADC_60,             "��� ��� 60�C");
    MAKE_MENU   (adc_70,        adc_80,         adc_60,         tmpr_calib,     NULL_ENTRY,     0,          ADC_70,             "��� ��� 70�C");
    MAKE_MENU   (adc_80,        adc_90,         adc_70,         tmpr_calib,     NULL_ENTRY,     0,          ADC_80,             "��� ��� 80�C");
    MAKE_MENU   (adc_90,        adc_100,        adc_80,         tmpr_calib,     NULL_ENTRY,     0,          ADC_90,             "��� ��� 90�C");
    MAKE_MENU   (adc_100,       adc_0,          adc_90,         tmpr_calib,     NULL_ENTRY,     0,          ADC_100,            "��� ��� 100�C");
MAKE_MENU       (save_changes,  NULL_ENTRY,     NULL_ENTRY,     NULL_ENTRY,     NULL_ENTRY,     0,          SAVE_CHANGES,       "��������� ���.");


/*========== FUNCTIONS ==========*/

void menu_init (void){
    strcpy(meas_ch_0.Text, dcts_meas[0].name);
    strcpy(meas_ch_1.Text, dcts_meas[1].name);
    strcpy(meas_ch_2.Text, dcts_meas[2].name);
    strcpy(meas_ch_3.Text, dcts_meas[3].name);
    strcpy(meas_ch_4.Text, dcts_meas[4].name);
    strcpy(meas_ch_5.Text, dcts_meas[5].name);
    strcpy(meas_ch_6.Text, dcts_meas[6].name);
    strcpy(meas_ch_7.Text, dcts_meas[7].name);
    strcpy(meas_ch_8.Text, dcts_meas[8].name);
    strcpy(meas_ch_9.Text, dcts_meas[9].name);
    strcpy(meas_ch_10.Text, dcts_meas[10].name);
    strcpy(meas_ch_11.Text, dcts_meas[11].name);
    strcpy(meas_ch_12.Text, dcts_meas[12].name);

    selectedMenuItem = &main_page;
}

void menuChange(menuItem* NewMenu){
    if (NewMenu != &NULL_ENTRY){
        selectedMenuItem = NewMenu;
    }
}
