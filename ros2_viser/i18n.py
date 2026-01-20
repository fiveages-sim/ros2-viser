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
        
        # FSM Panel
        "fsm_control": "FSM 控制",
        "current_state": "当前状态",
        "switch_pose": "切换姿态",
        
        # Gripper Panel
        "ee_control": "末端执行器控制",
        "open": "打开",
        "close": "关闭",
        "left_gripper": "左夹爪",
        "right_gripper": "右夹爪",
        "gripper": "夹爪",
        
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
        
        # FSM Panel
        "fsm_control": "FSM Control",
        "current_state": "Current State",
        "switch_pose": "Switch Pose",
        
        # Gripper Panel
        "ee_control": "EE Control",
        "open": "Open",
        "close": "Close",
        "left_gripper": "Left Gripper",
        "right_gripper": "Right Gripper",
        "gripper": "Gripper",
        
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


def t(key: str, default: str = None) -> str:
    """Convenience function to get translated text using global translator.
    
    Args:
        key: Translation key.
        default: Default value if key not found.
        
    Returns:
        Translated text.
    """
    return _global_translator(key, default)
