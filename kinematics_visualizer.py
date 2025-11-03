import tkinter as tk
import math
import serial

# Arm parameters
L1, L2, L3 = 200, 200, 100  # Lengths of the three links

# Global variables
ser = None

def try_connect_serial():
    """Try to connect to the Arduino and return True if successful"""
    global ser
    try:
        if ser is not None:
            ser.close()
        ser = serial.Serial('/dev/cu.usbmodem1101', 115200, timeout=1)
        return True
    except:
        return False

def getTheta1(v1):
    """Convert voltage back to theta1 angle"""
    return (v1 * 360 / 5) - 25

def getTheta2(v2):
    """Convert voltage back to theta2 angle"""
    return 160 - (v2 * 360 / 5)

def inverse_kinematics(x, y):
    """ Solves the inverse kinematics for the two-link arm """
    d = math.sqrt(x**2 + y**2)
    if d > (L1 + L2):
        return None  # Out of reach!
    
    # Check if end effector would go below wall
    # Calculate approximate end effector position after L3
    y_end = y - L3
    if y_end < -100:
        return None
    
    # Law of cosines
    cos_angle2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    angle2 = math.acos(max(min(cos_angle2, 1), -1))
    
    # Law of sines
    k1 = L1 + L2 * math.cos(angle2)
    k2 = L2 * math.sin(angle2)
    angle1 = math.atan2(y, x) - math.atan2(k2, k1)
    
    return math.degrees(angle1), math.degrees(angle2)

class TwoLinkArmApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Three-Link Arm Inverse Kinematics")
        self.root.configure(bg='#FFE0B2')
        self.theta1 = 100
        self.theta2 = 100
        self.theta3 = 45
        self.v1 = 0
        self.v2 = 0
        self.v3 = 0  # Value between 0-5V
        self.value4 = 72  # PWM angle based on link angles
        self.vacuum_state = 0  # 0 for off, 1 for on
        self.send_on_modify = False  # Toggle for auto-sending on modifications
        self.end_effector = [L1 + L2, 0]  # End effector position [x, y]
        
        # Create top frame for displays
        top_frame = tk.Frame(root, bg='#FFE0B2')
        top_frame.pack(pady=5)
        
        # Main movement display
        self.main_frame = tk.LabelFrame(top_frame, text='Arm Movement', bg='#FFE0B2', fg='#E65100')
        self.main_frame.pack(side='left', padx=5)
        self.canvas = tk.Canvas(self.main_frame, width=550, height=550, bg='#FFF3E0', highlightbackground='#FFB74D')
        self.canvas.pack(padx=5, pady=5)
        
        # Horizontal angle display
        self.horiz_frame = tk.LabelFrame(top_frame, text='Horizontal Angle', bg='#FFE0B2', fg='#E65100')
        self.horiz_frame.pack(side='left', padx=10)
        self.horiz_canvas = tk.Canvas(self.horiz_frame, width=320, height=320, bg='#FFF3E0', highlightbackground='#FFB74D')
        self.horiz_canvas.pack(padx=5, pady=5)
        
        # Create position display frame
        position_frame = tk.LabelFrame(root, text='Position Values', bg='#FFE0B2', fg='#E65100')
        position_frame.pack(pady=5, padx=10, fill='x')
        self.position_label = tk.Label(position_frame, text="Position: ", font=('Courier', 12), bg='#FFE0B2')
        self.position_label.pack(pady=5)
        
        # Create angle display frame
        angle_frame = tk.LabelFrame(root, text='Joint Angles', bg='#FFE0B2', fg='#1976D2')
        angle_frame.pack(pady=5, padx=10, fill='x')
        self.angle_label = tk.Label(angle_frame, text="Angles: ", font=('Courier', 12), bg='#FFE0B2', fg='#1976D2')
        self.angle_label.pack(pady=5)

        # Create command preview frame
        command_frame = tk.LabelFrame(root, text='Command Preview', bg='#FFE0B2', fg='#E65100')
        command_frame.pack(pady=5, padx=10, fill='x')
        self.command_label = tk.Label(command_frame, text="Command: ", font=('Courier', 12), bg='#FFE0B2')
        self.command_label.pack(pady=5)
        
        # Create Arduino feedback frame
        feedback_frame = tk.LabelFrame(root, text='Arduino Feedback', bg='#FFE0B2', fg='#2E7D32')
        feedback_frame.pack(pady=5, padx=10, fill='x')
        self.feedback_label = tk.Label(feedback_frame, text="Waiting for feedback...", font=('Courier', 12), bg='#FFE0B2', fg='#2E7D32')
        self.feedback_label.pack(pady=5)
        
        # Start checking for Arduino feedback
        self.root.after(1, self.check_arduino_feedback)
        
        # Bind mouse events for clock hand dragging
        self.horiz_canvas.bind('<Button-1>', self.start_clock_drag)
        self.horiz_canvas.bind('<B1-Motion>', self.drag_clock_hand)
        self.horiz_canvas.bind('<ButtonRelease-1>', self.stop_clock_drag)
        self.horiz_canvas.bind('<Enter>', self.on_horiz_enter)
        self.horiz_canvas.bind('<Leave>', self.on_horiz_leave)
        self.is_dragging_clock = False
        self.normal_bg = '#FFF3E0'  # Store normal background color
        self.horiz_angle_locked = False

        # Buttons
        button_frame = tk.Frame(root, bg='#FFE0B2')
        button_frame.pack(pady=5)
        
        tk.Button(button_frame, text='Send Position', command=self.send_position).pack(side='left', padx=5)
        self.vacuum_button = tk.Button(button_frame, text='Vacuum: OFF', command=self.toggle_vacuum)
        self.vacuum_button.pack(side='left', padx=5)
        self.send_modify_button = tk.Button(button_frame, text='Send on Modify: OFF', command=self.toggle_send_modify)
        self.send_modify_button.pack(side='left', padx=5)
        
        # Connection status and controls
        connection_frame = tk.Frame(root, bg='#FFE0B2')
        connection_frame.pack(pady=5)
        
        # Status indicator (small colored square)
        self.status_indicator = tk.Canvas(connection_frame, width=20, height=20, bg='#E57373', highlightthickness=0)
        self.status_indicator.pack(side='left', padx=5)
        
        # Connection status label
        self.status_label = tk.Label(connection_frame, text="Disconnected", bg='#FFE0B2')
        self.status_label.pack(side='left', padx=5)
        
        # Connect button
        self.connect_button = tk.Button(connection_frame, text='Connect', command=self.try_connect)
        self.connect_button.pack(side='left', padx=5)
        
        # Enable/Disable buttons
        enable_frame = tk.Frame(root, bg='#FFE0B2')
        enable_frame.pack(pady=5)
        
        self.enable_button = tk.Button(enable_frame, text='Enable System', command=self.enable_system, bg='#81C784', state='disabled')
        self.enable_button.pack(side='left', padx=5)
        self.disable_button = tk.Button(enable_frame, text='Disable System', command=self.disable_system, bg='#E57373', state='disabled')
        self.disable_button.pack(side='left', padx=5)
        
        # Try initial connection and start auto-reconnect
        self.try_connect()
        self.root.after(500, self.auto_reconnect)
        self.canvas.bind("<B1-Motion>", self.drag)
        self.draw_arm()
        

    
    def drag(self, event):
        x = event.x - 400  # Offset for center
        y = 400 - event.y  # Invert y-axis
        angles = inverse_kinematics(x, y)
        if angles:
            self.theta1, self.theta2 = angles
            self.v1 = ((self.theta1 + 25) / 360 ) * 5.3
            self.v2 = ((155 - self.theta2) / 360) * 5
            self.update_command()
            self.draw_arm()
            if self.send_on_modify:
                self.send_position()
                    
    
    def draw_arm(self, x1=None, y1=None, x2=None, y2=None, x3=None, y3=None):
        """Draw the robotic arm and environment on the canvas.
        If coordinates are not provided, calculate them from current angles."""
        # Clear canvas
        self.canvas.delete("all")
        
        # Constants for canvas layout
        CENTER_X = 400
        CENTER_Y = 400
        wall_y = 500
        
        # Draw environment elements
        # Target zone (grey rectangle)
        self.canvas.create_rectangle(100, 450, 200, 500, 
                                   fill='#E0E0E0', outline='#BDBDBD')
        # Wall and area below
        self.canvas.create_rectangle(0, wall_y, 800, 800, 
                                   fill='#FFE0B2', outline='')
        self.canvas.create_line(0, wall_y, 800, wall_y, 
                              fill='black', width=2)
        
        # Calculate arm positions if not provided
        if x1 is None:
            x1 = L1 * math.cos(math.radians(self.theta1))
            y1 = L1 * math.sin(math.radians(self.theta1))
            x2 = x1 + L2 * math.cos(math.radians(self.theta1 + self.theta2))
            y2 = y1 + L2 * math.sin(math.radians(self.theta1 + self.theta2))
            x3 = x2  # End effector x position
            y3 = y2 - L3  # End effector y position (straight down)
        
        # Draw the three arm links
        self.canvas.create_line(CENTER_X, CENTER_Y, 
                              CENTER_X + x1, CENTER_Y - y1, 
                              width=5, fill='#E65100')
        self.canvas.create_line(CENTER_X + x1, CENTER_Y - y1, 
                              CENTER_X + x2, CENTER_Y - y2, 
                              width=5, fill='#F57C00')
        self.canvas.create_line(CENTER_X + x2, CENTER_Y - y2, 
                              CENTER_X + x3, CENTER_Y - y3, 
                              width=5, fill='#FB8C00')
        
        # Draw joint balls
        ball_radius = 12
        # Base joint
        self.canvas.create_oval(400-ball_radius, 400-ball_radius, 
                               400+ball_radius, 400+ball_radius, 
                               fill='#FFB74D', outline='black', width=2)
        # First link joint
        self.canvas.create_oval(400+x1-ball_radius, 400-y1-ball_radius, 
                               400+x1+ball_radius, 400-y1+ball_radius, 
                               fill='#FFB74D', outline='black', width=2)
        # Second link joint (tip of link2)
        self.link2_ball = self.canvas.create_oval(400+x2-ball_radius, 400-y2-ball_radius, 
                               400+x2+ball_radius, 400-y2+ball_radius, 
                               fill='#FFB74D', outline='black', width=2)
        # Add hover bindings for link2 ball
        self.canvas.tag_bind(self.link2_ball, '<Enter>', self.on_link2_enter)
        self.canvas.tag_bind(self.link2_ball, '<Leave>', self.on_link2_leave)
        # End effector (tip of link3)
        self.canvas.create_oval(400+x3-ball_radius, 400-y3-ball_radius, 
                               400+x3+ball_radius, 400-y3+ball_radius, 
                               fill='#FF0000', outline='black', width=2)
      
        
        # Update position display
        position_text = f"Link 3 Tip Position:\n  X: {- x3:>6.1f}\n  Y: {y3:>6.1f}"
        self.position_label.config(text=position_text)
        
        # Update angle display
        angle_text = f"Joint Angles:\n  θ₁: {self.theta1:>6.1f}°\n  θ₂: {getTheta2(self.v2):>6.1f}°"
        self.angle_label.config(text=angle_text)

        # Calculate voltages from angles
        self.v1 = ((self.theta1 + 25) / 360 ) * 5.3
        self.v2 = ((155 - self.theta2) / 360) * 5  # Flipped voltage scaling
        
        # Update clock display if not locked
        if not self.horiz_angle_locked:
            self.update_clock_display(self._v3_to_angle(self.v3))
        
        # Calculate value4 from link angles

        second_link_angle = self.theta1 + self.theta2
        pwm_angle = (-90 - second_link_angle) % 360  # Keep angle positive
        if pwm_angle > 180:
            pwm_angle = pwm_angle - 360  # Convert to range -180 to 180
        # Flip the direction (0-180 to 180-0)
        pwm_angle = int(10 + abs(pwm_angle))
        self.value4 = pwm_angle  # Convert v3 (0-5) to angle (90-0)
    
    def toggle_vacuum(self):
        self.vacuum_state = 1 - self.vacuum_state  # Toggle between 0 and 1
        self.vacuum_button.config(text=f'Vacuum: {"ON" if self.vacuum_state else "OFF"}')
        self.update_command()
        if self.send_on_modify:
            self.send_position()
            
    def toggle_send_modify(self):
        self.send_on_modify = not self.send_on_modify
        self.send_modify_button.config(text=f'Send on Modify: {"ON" if self.send_on_modify else "OFF"}')

    def update_command(self):
        # Update command preview
        labeled_command = f"Command Values:\n  Angle1: {self.v1:.2f}V ({self.theta1:.1f}°)\n  Angle2: {self.v2:.2f}V ({self.theta2:.1f}°)\n  Link3: {self.v3:.1f}V\n  Value 4: {self.value4}°\n  Vacuum: {self.vacuum_state} ({'ON' if self.vacuum_state else 'OFF'})"
        self.command_label.config(text=labeled_command)
    
    
        
    def check_arduino_feedback(self):
        """Check for and display Arduino's serial feedback"""
        if ser and ser.in_waiting:
            try:
                feedback = ser.readline().decode().strip()
                if feedback.startswith("Angles"):
                    self.feedback_label.config(text=feedback)
            except:
                pass  # Ignore any decoding errors
        self.root.after(100, self.check_arduino_feedback)
    
    def _v3_to_angle(self, v3):
        # Convert v3 (0-5) to angle (180-0)
        return 180 - ((v3 / 5) * 180)
    
    def _angle_to_v3(self, angle):
        # Convert angle (180-0) to v3 (0-5)
        return 5 * (1 - (angle / 180))
        
    
    def update_clock_display(self, angle):
        self.theta3 = angle
        if self.horiz_angle_locked:
            return
        self._draw_angle_display(self.horiz_canvas, self.theta3, 'horizontal')
    
    def _draw_angle_display(self, canvas, angle, orientation):
        # Clear previous drawing
        canvas.delete('all')
        
        # Center coordinates for horizontal display
        cx, cy = 160, 160
        radius = 120
        
        # Draw horizontal reference line
        canvas.create_line(cx-radius, cy, cx+radius, cy, 
                         fill='#E65100', width=1, dash=(4, 4))
        
        # Draw center point
        canvas.create_oval(cx-3, cy-3, cx+3, cy+3, fill='#E65100')
        
        # Draw hand
        radian = math.radians(angle)  # 0° at 3 o'clock, 180° at 9 o'clock
        x = cx + radius * math.cos(radian)
        y = cy - radius * math.sin(radian)  # Subtract to invert Y
        canvas.create_line(cx, cy, x, y, fill='#FB8C00', width=3)
        
        # Draw draggable ball at end of hand
        ball_radius = 8
        canvas.create_oval(x-ball_radius, y-ball_radius,
                         x+ball_radius, y+ball_radius,
                         fill='#FFB74D', outline='black', width=2)
        
        # Display angle text
        canvas.create_text(cx, cy+radius+20, 
                         text=f'{angle:.1f}°', 
                         fill='#E65100',
                         font=('Arial', 10, 'bold'))

    def start_clock_drag(self, event):
        # Check if click is near the current ball position
        cx, cy = 160, 160  # Center of clock
        radius = 120
        current_angle = self._v3_to_angle(self.v3)
        current_radian = math.radians(current_angle)
        ball_x = cx + radius * math.cos(current_radian)
        ball_y = cy - radius * math.sin(current_radian)
        
        # Calculate distance from click to ball center
        dx = event.x - ball_x
        dy = event.y - ball_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Only start dragging if click is within 20 pixels of ball
        if distance <= 60:
            self.is_dragging_clock = True
    
    def toggle_horiz_lock(self):
        self.horiz_angle_locked = not self.horiz_angle_locked
        if self.horiz_angle_locked:
            # Store current values before locking
            self.last_v3 = self.v3
            self.last_angle = self._v3_to_angle(self.v3)
    
    def drag_clock_hand(self, event):
        if not self.is_dragging_clock or self.horiz_angle_locked:
            return

        # Get absolute mouse position relative to root window
        abs_x = event.widget.winfo_rootx() + event.x

        # Get main canvas position
        main_x = self.canvas.winfo_rootx()

        # Check if mouse is to the right of main canvas
        if abs_x <= main_x + self.canvas.winfo_width():
            return

        # Check if mouse is within horizontal angle canvas bounds
        if not (0 <= event.x <= 320 and 0 <= event.y <= 320):
            return

        # Calculate new angle from center to mouse position
        cx, cy = 160, 160  # Center of clock
        dx = event.x - cx
        dy = cy - event.y  # Invert y since canvas coordinates are inverted

        # Calculate angle in degrees
        angle = math.degrees(math.atan2(dy, dx))

        # Ensure angle is between 0 and 180
        if angle < 0:
            angle += 360
        if angle > 180:
            angle = 180

        # Only allow changes when unlocked
        if not self.horiz_angle_locked:
            # Update v3 and sliders
            self.v3 =  self._angle_to_v3(angle)
            self.update_clock_display(angle)
            self.update_command()

            if self.send_on_modify:
                self.send_position()

    def stop_clock_drag(self, event):
        self.is_dragging_clock = False
    
    def on_horiz_enter(self, event):
        # Unlock when mouse enters
        if self.horiz_angle_locked:
            self.toggle_horiz_lock()
        self.horiz_canvas.configure(bg='#A5D6A7')  # Light green
    
    def on_horiz_leave(self, event):
        # Lock when mouse leaves
        if not self.horiz_angle_locked:
            self.toggle_horiz_lock()
        self.horiz_canvas.configure(bg=self.normal_bg)
    
    def on_link2_enter(self, event):
        # Make link2 ball glow green on hover
        self.canvas.itemconfig(self.link2_ball, fill='#A5D6A7')
    
    def on_link2_leave(self, event):
        # Restore original color
        self.canvas.itemconfig(self.link2_ball, fill='#FFB74D')

    def enable_system(self):
        if ser is None:
            print("Not connected to Arduino")
            return
        try:
            print("Sending enable command")
            command = "enable\n"
            ser.write(command.encode())
        except:
            print("Failed to send command - connection lost")
            self.try_connect()  # Try to reconnect

    def disable_system(self):
        if ser is None:
            print("Not connected to Arduino")
            return
        try:
            print("Sending disable command")
            command = "disable\n"
            ser.write(command.encode())
        except:
            print("Failed to send command - connection lost")
            self.try_connect()  # Try to reconnect

    
    def try_connect(self):
        """Try to establish connection with Arduino"""
        if try_connect_serial():
            self.status_indicator.configure(bg='#81C784')  # Green
            self.status_label.configure(text='Connected')
            self.connect_button.configure(text='Reconnect')
            self.enable_button.configure(state='normal')
            self.disable_button.configure(state='normal')
            return True
        else:
            self.status_indicator.configure(bg='#E57373')  # Red
            self.status_label.configure(text='Disconnected')
            self.connect_button.configure(text='Connect')
            self.enable_button.configure(state='disabled')
            self.disable_button.configure(state='disabled')
            return False
            
    def auto_reconnect(self):
        """Automatically try to reconnect every 0.5 seconds if not connected"""
        self.try_connect()
        self.root.after(500, self.auto_reconnect)
    
    def send_position(self):
        if ser is None:
            print("Not connected to Arduino")
            return
            
        try:
            command = f"{self.v1:.2f},{self.v2:.2f},{self.v3:.2f},{self.value4},{self.vacuum_state}\n"
            print(f"Sending command: {command}")
            ser.write(command.encode())
            try_connect_serial()
        except:
            print("Failed to send command - connection lost")
            self.try_connect()  # Try to reconnect
    


if __name__ == "__main__":
    # Destroy any existing windows
    try:
        temp_root = tk.Tk()
        temp_root.destroy()
    except:
        pass
    
    root = tk.Tk()
    app = TwoLinkArmApp(root)
    root.mainloop()