#pragma once /* __BUTTON_MAP_H__ */

// 2023 Control Board button map

enum DR_BUTTON_MAP
{
    TR_BOT_CENTRIC = 5,
};

enum OP_BUTTON_MAP
{
    BUTTON_STOW = 8, // stow ?
    BUTTON_CLIMB = 2, // (pull climber down - does nothing without climb switch)
    SWITCH_CLIMB = 3, // (start climb == on)
    SWITCH_HI_LO = 4, // (low setpoint == off / high setpoint == on)
    SWITCH_SHOOTER = 5, // (shooter on == switch on)
    BUTTON_SHOOT = 6,
    BUTTON_HP = 7, // HP
    BUTTON_LAUNCH = 1, // launch pad ?
    BUTTON_INTAKE = 9,
    BUTTON_AUTO_SELECT = 9,
    BUTTON_EXHAUST = 10,
    BUTTON_RST_CONST = 10,
    BUTTON_GROUND = 11,
    BUTTON_AMP = 12,
    BUTTON_BREAK_MODE = 12
};