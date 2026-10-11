"""Internationalization (i18n) support for ROS2 Viser.

This module provides translation support for GUI labels and messages.
"""

from enum import Enum
from typing import Dict, Literal

# Supported languages
Language = Literal["zh", "en"]


class LanguageType(Enum):
    """Language type enumeration."""
    ZH = "zh"  # Chinese
    EN = "en"  # English


# Translation dictionary
_TRANSLATIONS: Dict[Language, Dict[str, str]] = {
    "zh": {
        # Display Control Panel
        "display_control": "显示控制",
        "show_visual": "显示 Visual",
        "show_collision": "显示 Collision",
        "language": "语言",
        "select_language": "选择语言",
        "marker_publish_mode": "末端指令模式",
        "continuous_publish": "连续发布",
        "single_publish": "单次发布",
        "send_marker_pose": "发送末端位置",
        
        # FSM Panel
        "fsm_control": "FSM 控制",
        "current_state": "当前状态",
        "switch_pose": "切换姿态",
        "fsm_home_confirm_title": "要切换到HOME吗？",
        
        # Gripper Panel
        "ee_control": "末端执行器控制",
        "open": "打开",
        "close": "关闭",
        "left_gripper": "左夹爪",
        "right_gripper": "右夹爪",
        "gripper": "夹爪",
        
        # Joint Panel
        "joint_control": "关节控制",
        "joint_category": "关节类别",
        "all": "全部",
        "category_body": "身体",
        "category_head": "头部",
        "category_left": "左臂",
        "category_right": "右臂",
        "category_left_hand": "左手",
        "category_right_hand": "右手",
        "send_joint_positions": "发送目标位置",
        "status": "状态",
        "waiting_for_joints": "等待关节初始化",
        "switch_to_joint_control_state": "请切换到支持关节控制的状态 (OCS2 或 MOVEJ)",
        "ready": "就绪",
        "body_link3_pose": "body_link3 @ base_footprint",
        "body_link3_pose_waiting": "等待 TF",
        "body_link3_pose_unavailable": "TF 不可用",
        
        # Waist control (lift + turn)
        "waist_control": "腰部控制",
        "waist_lifting_distance": "升降距离（米）",
        "waist_pose_x": "腰部 X（米）",
        "waist_pose_z": "腰部 Z（米）",
        "waist_pose_phi": "腰部 Phi（rad）",
        "waist_send_pose_relative": "发送相对 x/z/phi",
        "waist_send_pose_absolute": "发送绝对 x/z/phi",
        "waist_lifting_ratio": "升降速度比",
        "waist_actions": "腰部操作",
        "waist_step_up": "上升(距离)",
        "waist_step_down": "下降(距离)",
        "waist_hold_up": "上升(按住)",
        "waist_hold_down": "下降(按住)",
        "waist_turning_ratio": "旋转速度比",
        "waist_phi_ratio": "俯仰速度比例",
        "waist_hold_turn_left": "左转(按住)",
        "waist_hold_phi_forward": "前倾(按住)",
        "waist_hold_turn_right": "右转(按住)",
        "waist_hold_phi_backward": "后仰(按住)",
        
        # Hardware Panel
        "hardware_control": "驱动层配置",
        "hardware_system_detected": "硬件系统已检测",
        "m6_ccs_system_detected": "M6 CCS 系统已检测",
        "configure_m6_ccs_system": "配置 M6 CCS 系统",
        "m6_ccs_system_config_clicked": "M6 CCS 系统配置",
        "m6_ccs_configuration": "M6 CCS 配置",
        
        # Controller Panel
        "controller_control": "控制器配置",
        "controllers_detected": "已检测到控制器",
        "no_controllers_found": "未找到控制器",
        "configure_controller": "配置控制器",
        "configure": "配置",
        "refresh_controllers": "刷新控制器列表",
        
        # M6 CCS Parameter Names
        "param_ctrl_mode": "控制模式",
        "param_max_joint_speed": "最大关节速度",
        "param_max_joint_acceleration": "最大关节加速度",
        "param_cart_d_gains": "笛卡尔阻尼增益",
        "param_cart_k_gains": "笛卡尔刚度增益",
        "param_joint_d_gains": "关节阻尼增益",
        "param_joint_k_gains": "关节刚度增益",
        "param_left_dyn_param": "左末端动力学参数",
        "param_left_kine_param": "左末端运动学参数",
        "param_right_dyn_param": "右末端动力学参数",
        "param_right_kine_param": "右末端运动学参数",
        
        # Controller Parameter Names
        "param_duration": "单次持续时间",
        "param_interpolation_type": "插值类型",
        "param_tanh_scale": "Tanh 缩放",
        "param_position_threshold": "保持位置阈值",
        "param_trajectory_duration": "轨迹持续时间",
        "param_trajectory_blend_ratio": "轨迹混合比例",
        "param_lifting_duration": "单次持续时间",
        
        # Interpolation Type Options
        "interpolation_tanh": "Tanh",
        "interpolation_linear": "线性插值",
        "interpolation_doubles": "DoubleS插值",
        "interpolation_none": "不插值",
        
        # Control Mode Options
        "ctrl_mode_position": "位置控制",
        "ctrl_mode_joint_impedance": "关节阻抗控制",
        "ctrl_mode_cart_impedance": "笛卡尔阻抗控制",
        "configuration_parameter": "配置参数",
        "enter_config_value": "请输入配置值",
        "enable_feature": "启用功能",
        "confirm": "确认",
        "cancel": "取消",
        "configuration_saved": "配置已保存",
        "no_parameters_found": "未找到参数",
        "no_writable_parameters": "无可写参数",
        "no_changes": "无更改",
        
        # Messages
        "no_collision_meshes": "URDF 中没有碰撞网格",
        "collision_meshes_loaded": "碰撞网格已加载",
    },
    "en": {
        # Display Control Panel
        "display_control": "Display Control",
        "show_visual": "Show Visual",
        "show_collision": "Show Collision",
        "language": "Language",
        "select_language": "Select Language",
        "marker_publish_mode": "EE Target Mode",
        "continuous_publish": "Continuous Publish",
        "single_publish": "Single Publish",
        "send_marker_pose": "Send EE Pose",
        
        # FSM Panel
        "fsm_control": "FSM Control",
        "current_state": "Current State",
        "switch_pose": "Switch Pose",
        "fsm_home_confirm_title": "Confirm Switch to HOME",
        
        # Gripper Panel
        "ee_control": "EE Control",
        "open": "Open",
        "close": "Close",
        "left_gripper": "Left Gripper",
        "right_gripper": "Right Gripper",
        "gripper": "Gripper",
        
        # Joint Panel
        "joint_control": "Joint Control",
        "joint_category": "Joint Category",
        "all": "All",
        "category_body": "Body",
        "category_head": "Head",
        "category_left": "Left",
        "category_right": "Right",
        "category_left_hand": "Left Hand",
        "category_right_hand": "Right Hand",
        "send_joint_positions": "Send Joint Positions",
        "status": "Status",
        "waiting_for_joints": "Waiting for joints initialization",
        "switch_to_joint_control_state": "Please switch to joint control state (OCS2 or MOVEJ)",
        "ready": "Ready",
        "body_link3_pose": "body_link3 @ base_footprint",
        "body_link3_pose_waiting": "Waiting for TF",
        "body_link3_pose_unavailable": "TF unavailable",
        
        # Waist control (lift + turn)
        "waist_control": "Waist Control",
        "waist_lifting_distance": "Step Distance (m)",
        "waist_pose_x": "Waist X (m)",
        "waist_pose_z": "Waist Z (m)",
        "waist_pose_phi": "Waist Phi (rad)",
        "waist_send_pose_relative": "Send Relative x/z/phi",
        "waist_send_pose_absolute": "Send Absolute x/z/phi",
        "waist_lifting_ratio": "Lift Speed Ratio",
        "waist_actions": "Waist Actions",
        "waist_step_up": "Up (step)",
        "waist_step_down": "Down (step)",
        "waist_hold_up": "Up (hold)",
        "waist_hold_down": "Down (hold)",
        "waist_turning_ratio": "Turn Speed Ratio",
        "waist_phi_ratio": "Pitch Speed Ratio",
        "waist_hold_turn_left": "Turn Left (hold)",
        "waist_hold_phi_forward": "Tilt Forward (hold)",
        "waist_hold_turn_right": "Turn Right (hold)",
        "waist_hold_phi_backward": "Tilt Backward (hold)",
        
        # Hardware Panel
        "hardware_control": "Hardware Config",
        "hardware_system_detected": "Hardware system detected",
        "m6_ccs_system_detected": "M6 CCS system detected",
        "configure_m6_ccs_system": "Configure M6 CCS System",
        "m6_ccs_system_config_clicked": "M6 CCS system configuration",
        "m6_ccs_configuration": "M6 CCS Configuration",
        "configuration_parameter": "Configuration Parameter",
        "enter_config_value": "Enter configuration value",
        "enable_feature": "Enable Feature",
        
        # Controller Parameter Names
        "param_duration": "Duration",
        "param_interpolation_type": "Interpolation Type",
        "param_tanh_scale": "Tanh Scale",
        "param_hold_position_threshold": "Hold Position Threshold",
        "param_trajectory_duration": "Trajectory Duration",
        "param_trajectory_blend_ratio": "Trajectory Blend Ratio",
        
        # Interpolation Type Options
        "interpolation_tanh": "Tanh",
        "interpolation_linear": "Linear",
        "interpolation_doubles": "Doubles",
        "interpolation_none": "None",
        "confirm": "Confirm",
        "cancel": "Cancel",
        "configuration_saved": "Configuration saved",
        "no_parameters_found": "No parameters found",
        "no_writable_parameters": "No writable parameters",
        "no_changes": "No changes",
        
        # Controller Panel
        "controller_control": "Controller Config",
        "controllers_detected": "Controllers detected",
        "no_controllers_found": "No controllers found",
        "configure_controller": "Configure Controller",
        "configure": "Configure",
        "refresh_controllers": "Refresh Controllers",
        
        # Messages
        "no_collision_meshes": "No collision meshes in URDF",
        "collision_meshes_loaded": "Collision meshes loaded",
    },
}


class Translator:
    """Translation manager for GUI labels and messages.
    
    Example:
        ```python
        translator = Translator("zh")
        label = translator("display_control")  # Returns "显示控制"
        
        translator.set_language("en")
        label = translator("display_control")  # Returns "Display Control"
        ```
    """
    
    def __init__(self, language: Language = "zh"):
        """Initialize translator.
        
        Args:
            language: Language code ("zh" for Chinese, "en" for English).
        """
        self._language = language
        self._validate_language(language)
    
    def _validate_language(self, language: Language):
        """Validate language code."""
        if language not in _TRANSLATIONS:
            raise ValueError(f"Unsupported language: {language}. Supported: {list(_TRANSLATIONS.keys())}")
    
    def set_language(self, language: Language):
        """Change the current language.
        
        Args:
            language: Language code ("zh" or "en").
        """
        self._validate_language(language)
        self._language = language
    
    @property
    def language(self) -> Language:
        """Get current language."""
        return self._language
    
    def __call__(self, key: str, default: str = None) -> str:
        """Get translated text for a key.
        
        Args:
            key: Translation key.
            default: Default value if key not found. If None, returns the key itself.
            
        Returns:
            Translated text.
        """
        translations = _TRANSLATIONS.get(self._language, {})
        result = translations.get(key, default)
        if result is None:
            # If not found and no default, return the key itself
            return key
        return result
    
    def get(self, key: str, default: str = None) -> str:
        """Get translated text for a key (alias for __call__).
        
        Args:
            key: Translation key.
            default: Default value if key not found.
            
        Returns:
            Translated text.
        """
        return self(key, default)
    
    def add_translation(self, key: str, zh_text: str, en_text: str):
        """Add or update a translation.
        
        Args:
            key: Translation key.
            zh_text: Chinese text.
            en_text: English text.
        """
        _TRANSLATIONS["zh"][key] = zh_text
        _TRANSLATIONS["en"][key] = en_text


# Global translator instance (default to Chinese)
_global_translator = Translator("zh")


def get_translator() -> Translator:
    """Get the global translator instance.
    
    Returns:
        Global Translator instance.
    """
    return _global_translator


def set_global_language(language: Language):
    """Set the global language.
    
    Args:
        language: Language code ("zh" or "en").
    """
    _global_translator.set_language(language)


def get_text_in_language(language: Language, key: str, default: str = None) -> str:
    """Get translated text for a key in a specific language (ignores global setting).

    Args:
        language: Language code ("zh" or "en").
        key: Translation key.
        default: Default value if key not found.

    Returns:
        Translated text.
    """
    translations = _TRANSLATIONS.get(language, {})
    result = translations.get(key, default)
    if result is None:
        return key
    return result


def t(key: str, default: str = None) -> str:
    """Convenience function to get translated text using global translator.
    
    Args:
        key: Translation key.
        default: Default value if key not found.
        
    Returns:
        Translated text.
    """
    return _global_translator(key, default)
