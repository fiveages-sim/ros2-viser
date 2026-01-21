"""Base Modal Dialog Class for ROS2 Viser."""

import logging
from typing import Optional, Callable, Dict, Any
from abc import ABC, abstractmethod

import viser

from ..i18n import get_translator

logger = logging.getLogger(__name__)


class BaseModalDialog(ABC):
    """Base class for modal dialogs using Viser's native modal API.
    
    Example:
        ```python
        class MyDialog(BaseModalDialog):
            def _build_content(self, gui):
                gui.add_text("Input", initial_value="")
        ```
    """
    
    def __init__(
        self,
        server: viser.ViserServer,
        title: str,
        on_confirm: Optional[Callable[[Dict[str, Any]], None]] = None,
        on_cancel: Optional[Callable[[], None]] = None
    ):
        """Initialize base modal dialog.
        
        Args:
            server: Viser server instance.
            title: Dialog title.
            on_confirm: Callback function called when confirm button is clicked.
                       Receives a dict of input values.
            on_cancel: Callback function called when cancel button is clicked.
        """
        self.server = server
        self.title = title
        self.on_confirm = on_confirm
        self.on_cancel = on_cancel
        self.translator = get_translator()
        self._modal_handle: Optional[viser.GuiModalHandle] = None
        self._inputs: Dict[str, Any] = {}
    
    def show(self):
        """Show the modal dialog."""
        if self._modal_handle is not None:
            logger.warning(f"Dialog '{self.title}' is already open")
            return
        
        try:
            # Use server's gui.add_modal (works for all connected clients)
            self._modal_handle = self.server.gui.add_modal(self.title)
            
            with self._modal_handle:
                # Build dialog content
                self._build_content(self.server.gui)
                
                # Add buttons
                confirm_btn = self.server.gui.add_button(
                    self.translator("confirm"),
                    color="green"
                )
                confirm_btn.on_click(lambda _: self._handle_confirm())
                
                cancel_btn = self.server.gui.add_button(
                    self.translator("cancel"),
                    color="red"
                )
                cancel_btn.on_click(lambda _: self._handle_cancel())
            
            logger.info(f"Modal dialog '{self.title}' opened")
        except Exception as e:
            logger.error(f"Failed to show dialog '{self.title}': {e}", exc_info=True)
            self.close()
    
    def close(self):
        """Close the modal dialog."""
        try:
            if self._modal_handle is not None:
                self._modal_handle.close()
                self._modal_handle = None
            
            self._inputs = {}
            logger.debug(f"Modal dialog '{self.title}' closed")
        except Exception as e:
            logger.warning(f"Error closing dialog '{self.title}': {e}")
    
    @abstractmethod
    def _build_content(self, gui):
        """Build dialog content. Must be implemented by subclasses.
        
        Args:
            gui: Viser GUI API instance (server.gui or client.gui).
                 Use this to add UI elements and store handles in self._inputs.
        """
        pass
    
    def _handle_confirm(self):
        """Handle confirm button click."""
        try:
            values = self._get_input_values()
            if self.on_confirm is not None:
                self.on_confirm(values)
            self.close()
        except Exception as e:
            logger.error(f"Error handling dialog confirm: {e}", exc_info=True)
    
    def _handle_cancel(self):
        """Handle cancel button click."""
        try:
            if self.on_cancel is not None:
                self.on_cancel()
            self.close()
        except Exception as e:
            logger.error(f"Error handling dialog cancel: {e}", exc_info=True)
    
    def _get_input_values(self) -> Dict[str, Any]:
        """Get values from all dialog inputs."""
        values = {}
        for name, handle in self._inputs.items():
            try:
                if hasattr(handle, 'value'):
                    values[name] = handle.value
                elif hasattr(handle, 'checked'):
                    values[name] = handle.checked
                else:
                    values[name] = None
            except Exception as e:
                logger.warning(f"Failed to get value for input '{name}': {e}")
                values[name] = None
        return values
    
    @property
    def is_open(self) -> bool:
        """Check if dialog is currently open."""
        return self._modal_handle is not None
