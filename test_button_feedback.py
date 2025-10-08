#!/usr/bin/env python3
"""
Test script to verify button visual feedback and reset status logging fixes
"""

import tkinter as tk
import time
from hexapod.gui import Colors, ControlPanel

class TestControlPanel:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Button Feedback Test")
        self.root.geometry("600x400")
        self.root.configure(bg=Colors.BACKGROUND)

        # Create control panel
        self.control_panel = ControlPanel(self.root, width=580, height=350)
        self.control_panel.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Set up test callbacks
        self.control_panel.set_callbacks(
            movement_callbacks={
                'forward': lambda: self.log_action("MOVEMENT: Forward"),
                'backward': lambda: self.log_action("MOVEMENT: Backward"),
                'left': lambda: self.log_action("MOVEMENT: Left"),
                'right': lambda: self.log_action("MOVEMENT: Right"),
                'turn_left': lambda: self.log_action("MOVEMENT: Turn Left"),
                'turn_right': lambda: self.log_action("MOVEMENT: Turn Right"),
            },
            action_callbacks={
                'start': lambda: self.log_action("ACTION: Start"),
                'stop': lambda: self.log_action("ACTION: Stop"),
                'crouch': lambda: self.log_action("ACTION: Crouch"),
                'stand': lambda: self.log_action("ACTION: Stand"),
            },
            system_callbacks={
                'reset': lambda: self.log_action("SYSTEM: Reset"),
                'settings': lambda: self.log_action("SYSTEM: Settings"),
            }
        )

        # Test instructions
        instructions = tk.Label(
            self.root,
            text="Test Instructions:\n1. Click movement buttons (WASD, QE) - should flash green briefly\n2. Click action buttons (START, STOP) - should flash green briefly\n3. Click system buttons (RESET, SETTINGS) - should flash orange briefly\n4. Check status log for proper timestamped messages",
            bg=Colors.BACKGROUND,
            fg=Colors.TEXT_PRIMARY,
            font=("Arial", 10),
            justify=tk.LEFT
        )
        instructions.pack(fill=tk.X, padx=10, pady=(0, 10))

    def log_action(self, message):
        """Log action to status"""
        print(f"📝 Logged: {message}")
        self.control_panel.update_status(message)

    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    test_app = TestControlPanel()
    test_app.run()