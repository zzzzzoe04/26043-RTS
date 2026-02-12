"""
ESP32 Assessment GUI (Tkinter) 
this version builds on the save only version(V3), adding a finger activation map with wireframe hands(V5) and a display of previous trial data(V6)

CSV format per trial:
Row1: "UserID: X, SessionID: Y, Date: Z, Hand: H"
Row2: "RandomDelay_ms: <value>"
Row3: "Combination", c0,c1,...,c9
Row4: "ActivatedDigit", a0,a1,...,a9
Row5: header -> t_ms, f0, f1, ..., f9
Row6+: sample rows: t_ms, f0..f9

"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial, serial.tools.list_ports
import threading, time, datetime, os, csv, random, queue
import pandas as pd
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter


# CONFIG 
BAUDRATE = 921600 # must match ESP and be high enough for 1000Hz data streaming
OUT_DIR = "esp_data" # name of folder for data saving, created in the same folder that the script is running from
os.makedirs(OUT_DIR, exist_ok=True) # make folder to hold trial data
CHECK_RAW = False  # if True, MAKE SURE TO DELETE EXISTING FILE FIRST - capture raw serial data to raw_serial_dump.bin for communication debugging

# random.seed(0) # fix seed for randomization so the sequence is always the same at startup - useful for debugging but not for real use
# 45 digit combinations
COMBOS = [(1, 1, 1, 1, 1, 0, 0, 0, 0, 0, ), 
(0, 1, 1, 1, 1, 0, 0, 0, 0, 0, ),
(1, 1, 1, 1, 0, 0, 0, 0, 0, 0, ),
(1, 1, 1, 0, 0, 0, 0, 0, 0, 0, ),
(0, 1, 1, 1, 0, 0, 0, 0, 0, 0, ),
(0, 0, 1, 1, 1, 0, 0, 0, 0, 0, ),
(1, 1, 0, 0, 0, 0, 0, 0, 0, 0, ),
(0, 1, 1, 0, 0, 0, 0, 0, 0, 0, ),
(0, 0, 1, 1, 0, 0, 0, 0, 0, 0, ),
(0, 0, 0, 1, 1, 0, 0, 0, 0, 0, ),
(1, 0, 0, 0, 0, 0, 0, 0, 0, 0, ),
(0, 1, 0, 0, 0, 0, 0, 0, 0, 0, ),
(0, 0, 1, 0, 0, 0, 0, 0, 0, 0, ),
(0, 0, 0, 1, 0, 0, 0, 0, 0, 0, ),
(0, 0, 0, 0, 1, 0, 0, 0, 0, 0, ),
(0, 0, 0, 0, 0, 1, 1, 1, 1, 1, ),
(0, 0, 0, 0, 0, 0, 1, 1, 1, 1, ),
(0, 0, 0, 0, 0, 1, 1, 1, 1, 0, ),
(0, 0, 0, 0, 0, 1, 1, 1, 0, 0, ),
(0, 0, 0, 0, 0, 0, 1, 1, 1, 0, ),
(0, 0, 0, 0, 0, 0, 0, 1, 1, 1, ),
(0, 0, 0, 0, 0, 1, 1, 0, 0, 0, ),
(0, 0, 0, 0, 0, 0, 1, 1, 0, 0, ),
(0, 0, 0, 0, 0, 0, 0, 1, 1, 0, ),
(0, 0, 0, 0, 0, 0, 0, 0, 1, 1, ),
(0, 0, 0, 0, 0, 1, 0, 0, 0, 0, ),
(0, 0, 0, 0, 0, 0, 1, 0, 0, 0, ),
(0, 0, 0, 0, 0, 0, 0, 1, 0, 0, ),
(0, 0, 0, 0, 0, 0, 0, 0, 1, 0, ),
(0, 0, 0, 0, 0, 0, 0, 0, 0, 1, ),
(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, ),
(0, 1, 1, 1, 1, 1, 1, 1, 1, 0, ),
(1, 1, 1, 1, 0, 0, 1, 1, 1, 1, ),
(1, 1, 1, 0, 0, 0, 0, 1, 1, 1, ),
(0, 1, 1, 1, 0, 0, 1, 1, 1, 0, ),
(0, 0, 1, 1, 1, 1, 1, 1, 0, 0, ),
(1, 1, 0, 0, 0, 0, 0, 0, 1, 1, ),
(0, 1, 1, 0, 0, 0, 0, 1, 1, 0, ),
(0, 0, 1, 1, 0, 0, 1, 1, 0, 0, ),
(0, 0, 0, 1, 1, 1, 1, 0, 0, 0, ),
(1, 0, 0, 0, 0, 0, 0, 0, 0, 1, ),
(0, 1, 0, 0, 0, 0, 0, 0, 1, 0, ),
(0, 0, 1, 0, 0, 0, 0, 1, 0, 0, ),
(0, 0, 0, 1, 0, 0, 1, 0, 0, 0, ),
(0, 0, 0, 0, 1, 1, 0, 0, 0, 0, ),
]



# Serial manager - for communication with ESP32 _________________________________________________________________
class SerialManager:
    def __init__(self):
        self.ser = None
        self.port = None
        self.baud = BAUDRATE
        self.alive = False
        self.reader_thread = None
        # Two queues: one for status/message lines, one for sample lines
        self.msg_queue = queue.Queue()
        self.sample_queue = queue.Queue()

    def open(self, port):
        self.close()
        try:
            self.ser = serial.Serial(port, self.baud, timeout=0.1)
            time.sleep(2.0)  # allow device to boot
        except Exception as e:
            return False, str(e)
        self.port = port
        self.alive = True
        self.reader_thread = threading.Thread(target=self._reader, daemon=True)
        self.reader_thread.start()
        return True, ""

    def close(self):
        self.alive = False
        if self.ser:
            try:
                self.ser.close()
            except:
                pass
            self.ser = None

    def send_line(self, text):
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("Serial not open")
        self.ser.write((text + "\n").encode())

    def _reader(self):
        import struct
        import time
        import queue

        if CHECK_RAW:
            raw_log = open("raw_serial_dump.bin", "ab")

        buf = bytearray()

        PACKET_HEADER = b'\xAA\x55'
        PACKET_TYPE_SAMPLE = 0x01
        PACKET_LEN = 30  # 2 header + 1 type + 27 payload
        # last = time.time() # temp for debugging
        SAMPLE_STRUCT = struct.Struct("<I10HBH")  # 27 bytes

        while self.alive and self.ser:
            try:
                raw = self.ser.read(PACKET_LEN*10)
                if not raw:
                    time.sleep(0.001)
                    continue
                if CHECK_RAW:
                    # OPTIONAL RAW CAPTURE FOR DEBUGGING
                    try:
                        raw_log.write(raw)
                    except Exception as e:
                        print("RAW LOGGING ERROR:", e)

                buf.extend(raw) 

                while True:
                    # 1. Handle ASCII messages (must start with '#' AND contain newline) 
                    if buf and buf[0] == ord('#'):
                        nl = buf.find(b'\n', 1)
                        if nl == -1:
                            break  # wait for full line

                        if nl > 128: # ASCII line too long
                            print("ASCII DESYNC: dropping 1 byte:", buf[:4])
                            del buf[0]  # not a real message → resync
                            continue

                        try:
                            line = buf[:nl].decode("ascii").strip()
                            # log raw messages with timestamp 
                            with open("raw_msg_tap.log", "a") as f:
                                f.write(f"{time.time():.6f} MSG {line}\n")
                        except UnicodeDecodeError:
                            print("ASCII DECODING ERROR: dropping 1 byte:", buf[:4])
                            del buf[0]
                            continue

                        del buf[:nl + 1]

                        try:
                            self.msg_queue.put_nowait(line[1:])
                            # temp for debugging
                            #if time.time() - last > 0.02:
                                # print("READER STALL:", time.time() - last)
                            #    last = time.time()
                        except queue.Full:
                            pass
                        continue

                    # 2) BINARY SAMPLE PACKET                    
                    if len(buf) >= PACKET_LEN and buf[0:2] == PACKET_HEADER:
                        pkt_type = buf[2]

                        if pkt_type != PACKET_TYPE_SAMPLE:
                            # Unknown packet → resync by shifting 1 byte
                            print("BAD TYPE:", pkt_type, "dropping 1 byte:", buf[:4])
                            del buf[0]
                            continue

                        pkt = buf[:PACKET_LEN]
                        del buf[:PACKET_LEN]

                        try:
                            payload = pkt[3:]  # 27 bytes
                            unpacked = SAMPLE_STRUCT.unpack(payload)

                            t_ms = unpacked[0]
                            forces = list(unpacked[1:11])
                            active_digit = unpacked[11]
                            rand_delay = unpacked[12]

                            try:
                                self.sample_queue.put_nowait(
                                    (t_ms, forces, active_digit, rand_delay)
                                )
                            except queue.Full:
                                print("QUEUE FULL at", time.time())
                                pass

                        except struct.error:
                            # corrupted frame → drop and resync
                            continue

                        continue

                    # 3) RESYNC: discard one byte
                    if buf:
                        if len(buf) < 3:
                            break 
                        print("RESYNC: dropping 1 byte:", buf[:4])
                        del buf[0]
                    else:
                        break    

            except Exception as e:
                try:
                    self.msg_queue.put_nowait(f"SERIAL_ERROR:{e}")
                except queue.Full:
                    pass
                time.sleep(0.05)
        if CHECK_RAW:
            raw_log.close()

    def get_msg_nowait(self, timeout=0.01):
        try:
            return self.msg_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def get_sample_nowait(self, timeout=0.01):
        try:
            return self.sample_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def flush_all(self):
        # empty queues (used before handshake)
        while not self.msg_queue.empty():
            try: self.msg_queue.get_nowait()
            except: break
        while not self.sample_queue.empty():
            try: self.sample_queue.get_nowait()
            except: break

# GUI App ____________________________________________________________________________________________
class AssessmentApp:
    def __init__(self, root):
        self.root = root
        root.title("ESP32 Assessment GUI (save-only)")
        root.configure(bg="white")

        self.serial = SerialManager()
        self.session_active = False
        self.trial_thread = None
        self.stop_requested = False
        self.prev_stop_occurred = False
        self.trial_counter = 0

        # GUI variables
        self.selected_port = tk.StringVar()
        self.status_var = tk.StringVar(value="Not connected")
        self.user_id = tk.StringVar()
        self.session_id = tk.StringVar()
        self.date_str = tk.StringVar(value=datetime.date.today().isoformat())
        self.hand_choice = tk.StringVar(value="Both")
        self.last_trial_info = tk.StringVar(value="No trials yet                                                 ") # the spaces help with GUI spacing 
        self.num_trials_var = tk.IntVar(value=0)   # 0 means unlimited unless specified
        self.num_trials = 0
        # build UI
        self._build_ui()

        # update timers
        self.root.after(100, self._periodic_tasks)

    def _build_ui(self):
        
        frm = ttk.Frame(self.root, padding=6, style="Card.TFrame")
        frm.grid(row=0, column=0, sticky="nsew")
        frm.columnconfigure(0, weight=1)

        # Serial Controls 
        serial_card = ttk.LabelFrame(frm, text="Serial & Session Controls")
        serial_card.grid(row=0, column=0, sticky="we", pady=(0, 6))
        serial_card.columnconfigure(1, weight=1)

        ttk.Label(serial_card, text="Port:").grid(row=0, column=0, sticky="w", padx=4, pady=4)
        self.port_combo = ttk.Combobox(serial_card, textvariable=self.selected_port, width=28)
        self.port_combo.grid(row=0, column=1, sticky="we", padx=4)

        ttk.Button(serial_card, text="Refresh", command=self._refresh_ports).grid(row=0, column=2, padx=4)
        ttk.Button(serial_card, text="Connect", command=self._connect).grid(row=0, column=3, padx=4)
        ttk.Button(serial_card, text="Disconnect", command=self._disconnect).grid(row=0, column=4, padx=4)

        # Session Inputs 
        input_card = ttk.LabelFrame(frm, text="Session Inputs")
        input_card.grid(row=1, column=0, sticky="we", pady=(0, 6))
        input_card.columnconfigure(1, weight=1)

        ttk.Label(input_card, text="User ID").grid(row=0, column=0, sticky="w", padx=4, pady=4)
        ttk.Entry(input_card, textvariable=self.user_id).grid(row=0, column=1, sticky="we", padx=4)

        ttk.Label(input_card, text="Session ID").grid(row=1, column=0, sticky="w", padx=4, pady=4)
        ttk.Entry(input_card, textvariable=self.session_id).grid(row=1, column=1, sticky="we", padx=4)

        ttk.Label(input_card, text="Date").grid(row=2, column=0, sticky="w", padx=4, pady=4)
        ttk.Entry(input_card, textvariable=self.date_str).grid(row=2, column=1, sticky="we", padx=4)

        ttk.Label(input_card, text="Hand").grid(row=3, column=0, sticky="w", padx=4, pady=4)
        ttk.OptionMenu(input_card, self.hand_choice, "Both", "Left", "Right", "Both").grid(row=3, column=1, sticky="we", padx=4)

        ttk.Label(input_card, text="Number of Trials").grid(row=4, column=0, sticky="w", padx=4, pady=4)
        ttk.Entry(input_card, textvariable=self.num_trials_var).grid(row=4, column=1, sticky="we", padx=4)

        # Start / Stop
        action_card = ttk.Frame(frm, padding=4)
        action_card.grid(row=2, column=0, sticky="we", pady=(0, 12))
        action_card.columnconfigure(2, weight=1)

        ttk.Button(action_card, text="Start Session", command=self.start_session).grid(row=0, column=0, padx=4)
        ttk.Button(action_card, text="Stop Session", command=self.stop_session).grid(row=0, column=1, padx=4)
        ttk.Label(action_card, text="Status:").grid(row=0, column=2, sticky="e", padx=6)
        ttk.Label(action_card, textvariable=self.status_var).grid(row=0, column=3, sticky="w")

        ttk.Button(action_card, text="View Results", command=self.open_results_window)\
            .grid(row=1, column=0, columnspan=2, pady=(8, 0))

        #  Hands Canvas 
        hand_card = ttk.LabelFrame(frm, text="Finger Activation Map")
        hand_card.grid(row=3, column=0, sticky="we", pady=(0, 12))

        self.canvas = tk.Canvas(hand_card, width=520, height=200, bg="#FFFFFF", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=10, pady=10)

        self._draw_wireframe_hands()

        # Bottom Panels
        bottom = ttk.Frame(frm)
        bottom.grid(row=4, column=0, sticky="nsew")
        bottom.columnconfigure(0, weight=0)
        bottom.columnconfigure(1, weight=1)

        info_box = ttk.LabelFrame(bottom, text="Last Trial")
        info_box.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        ttk.Label(info_box, textvariable=self.last_trial_info, anchor="w", justify="left").pack(fill="both", padx=6, pady=6)

        log_card = ttk.LabelFrame(bottom, text="Messages")
        log_card.grid(row=0, column=1, sticky="nsew", padx=6, pady=6)
        self.log_text = tk.Text(log_card, height=12, wrap="word", bg="#FFFFFF", fg="#111113", relief="solid", bd=1)
        self.log_text.pack(fill="both", padx=6, pady=6)
        self.log_text.configure(state="disabled")

        
        self._refresh_ports()

    # Wireframe Hands Drawing
    def _draw_wireframe_hands(self):
        c = self.canvas
        self.finger_nodes = {}

        # Base geometry
        left_offset = 180
        right_offset = 500
        palm_top = 125
        palm_width = 85
        palm_height = 100
        node_rx = 8  # fingertip circle radius
        node_ry = 35


        def draw_palm(x_offset):
            return c.create_oval(
                x_offset, palm_top,
                x_offset + palm_width, palm_top + palm_height,
                outline="#FFFFFF", width=2
            )


        def draw_fingertips(x_offset, side_prefix):
            spacing = palm_width / 4
            top_y = palm_top - 50
            thumb_y = palm_top + palm_height * 0.10  # lower on palm
            thumb_x = x_offset - 15 if side_prefix == "R" else x_offset + palm_width + 15

            # Thumb (digit 0 or 5)
            thumb_node = c.create_oval(
                thumb_x - node_rx, thumb_y - (node_ry*0.8),
                thumb_x + node_rx, thumb_y + (node_ry*0.8),
                fill="white", outline="#A1A1AA", width=2
            )
            if side_prefix == "R":
                self.finger_nodes[f"{side_prefix}5"] = thumb_node
            else:
                self.finger_nodes[f"{side_prefix}4"] = thumb_node

            # Fingers 1–4: Top arc
            for i in range(1, 5):
                cx = x_offset + spacing * (i - 0.5)
                if side_prefix == "R":
                    vert_shift = [2, 3, 2, 0]
                if side_prefix == "L":
                    vert_shift = [0, 2, 3, 2]
                cy = top_y
                node = c.create_oval(
                    cx - node_rx, cy - (node_ry+(vert_shift[i-1]*7)),
                    cx + node_rx, cy + (node_ry-(vert_shift[i-1]*5)),
                    fill="white", outline="#A1A1AA", width=2
                )
                if side_prefix == "L":
                    self.finger_nodes[f"{side_prefix}{i-1}"] = node
                if side_prefix == "R":
                    self.finger_nodes[f"{side_prefix}{i+5}"] = node

        # Draw both hands
        draw_palm(left_offset)
        draw_fingertips(left_offset, "L")

        draw_palm(right_offset)
        draw_fingertips(right_offset, "R")
  
    # Update Fingers (public)
    def update_possible_digits(self, possible_list):
        """Set finger nodes that are part of the combo (green-ish highlight)."""
        # Clear all
        for node in self.finger_nodes.values():
            self.canvas.itemconfig(node, fill="white", outline="#A1A1AA")

        # Highlight possible digits (list of ints 0-9)
        for d in possible_list:
            side = "L" if d < 5 else "R"
            idx = d # if d < 5 else d - 5
            node_id = self.finger_nodes[f"{side}{idx}"]
            self.canvas.itemconfig(node_id,
                fill="#EFF6FF",
                outline="#3B82F6"
            )

    # this doesnt get called anywhere bc we dont want the user to know which digit will vibrate but its here in case we want to see the active digit during testing
    def update_active_digit(self, digit):
        """Highlight the currently active digit (if any)."""
        if digit is None or digit < 0 or digit > 9:
            return

        side = "L" if digit < 5 else "R"
        idx = digit if digit < 5 else digit - 5
        node = self.finger_nodes[f"{side}{idx}"]

        # No glow effect per instructions — just solid accent
        self.canvas.itemconfig(node, fill="#3B82F6", outline="#1D4ED8")

    # logging
    def log(self, *args):
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        line = " ".join(str(a) for a in args)
        msg = f"[{ts}] {line}\n"
        # print to terminal too
        print(msg.strip())
        self.log_text.configure(state="normal")
        self.log_text.insert("end", msg)
        self.log_text.see("end")
        self.log_text.configure(state="disabled")
    # port refresh
    def _refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        names = [p.device for p in ports]
        self.port_combo['values'] = names
        if names and not self.selected_port.get():
            self.selected_port.set(names[1])
    # connect
    def _connect(self):
        port = self.selected_port.get().strip()
        if not port:
            messagebox.showwarning("Serial", "Select a serial port first.")
            return
        ok, err = self.serial.open(port)
        if not ok:
            messagebox.showerror("Serial", f"Failed to open port {port}:\n{err}")
            self.log("Failed to open port", port, err)
            return
        self.log("Connected to", port)
        self.status_var.set("Connected, starting handshake")
        # handshake in background
        threading.Thread(target=self._handshake_thread, daemon=True).start()
    # disconnect
    def _disconnect(self):
        self.serial.close()
        self.status_var.set("Disconnected")
        self.log("Serial disconnected")

    # handshake
    def _handshake_thread(self):
        # send ASSESS until OK received
        self.serial.flush_all()
        tries = 0
        while tries < 400:
            try:
                self.serial.send_line("ASSESS")
            except Exception as e:
                self.log("Send failed:", e)
                self.status_var.set("Send failed")
                return
            # wait short window for OK
            start = time.time()
            got_ok = False
            while time.time() - start < 0.4:
                ln = self.serial.get_msg_nowait(timeout=0.05)
                if ln:
                    # show high-level messages but not samples
                    self.log("ESP ->", ln)
                    if ln.strip() == "OK":
                        got_ok = True
                        break
                else:
                    time.sleep(0.01)
            if got_ok:
                self.status_var.set("Handshake OK")
                self.log("Handshake OK")
                return
            tries += 1
            time.sleep(0.05)
        self.status_var.set("Handshake timed out")
        self.log("Handshake timed out")

    # Session control
    def start_session(self):
        if not self.serial.ser:
            messagebox.showerror("Serial", "Connect to serial port first.")
            return
        if self.session_active:
            messagebox.showinfo("Session", "Session already running.")
            return
        if not self.user_id.get().strip() or not self.session_id.get().strip():
            if not messagebox.askyesno("Missing info", "UserID or SessionID empty. Continue?"):
                return

        self.session_active = True
        self.stop_requested = False
        self.num_trials = int(self.num_trials_var.get())
        self.trial_counter = 0
        self.status_var.set("Session running")
        self.log("Session started")
        if self.prev_stop_occurred == True:
            self.serial.send_line("SESSION START")
        self.trial_thread = threading.Thread(target=self._session_loop, daemon=True)
        self.trial_thread.start()

    def stop_session(self):
        if not self.session_active:
            self.log("Stop requested but session not running.")
            return
        self.stop_requested = True
        self.status_var.set("Stopping after current trial")
        self.log("Stop requested")
        self.prev_stop_occurred = True
                   # update rectangle colors
                   # this is an artifact from previous code versions - update using update_possible_digits instead
        for i in range(10):
            #self.canvas.itemconfig(self.rects[i], fill="grey")
            possible_list = []
            self.update_possible_digits(possible_list)

    def _session_loop(self):
        # main loop: wait for WAIT_COMBO, send combo, collect sample lines until TRIAL_DONE, save CSV
        while self.session_active and not self.stop_requested:
            self.log("Waiting for WAIT_COMBO from ESP...")
            self.serial.send_line("WAIT_COMBO")
            got_wait = False
            wait_start = time.time()
            while time.time() - wait_start < 10:  # 10s timeout waiting for WAIT_COMBO
                ln = self.serial.get_msg_nowait(timeout=0.2)
                if ln:
                    self.log("ESP ->", ln)
                    if ln.strip() == "OK":
                        self.serial.send_line("WAIT_COMBO")
                    if ln.strip() == "WAIT_COMBO":
                        got_wait = True
                        break
                time.sleep(0.01)
            if not got_wait:
                self.log("Timeout waiting for WAIT_COMBO; retrying...")
                self.serial.send_line("WAIT_COMBO")
                continue

            # choose combo and send
            hand_mode = self.hand_choice.get()
            subset = [] # initialize subset list
            for v in COMBOS: # loop through all combos
                if hand_mode == "Left":
                    # only left hand digits allowed (indices 0..4); ensure no right digits active
                    if any(v[5:]): continue # skip over combinations that use right hand digits (indices 5-9)
                    subset.append(v) # otherwise append to the subset
                elif hand_mode == "Right":
                    if any(v[0:5]): continue # skip over any combos with active digits in spot 0,1,2,3, or 4
                    subset.append(v)
                else:
                    subset.append(v) # otherwise the hand mode is "both" so all combos are valid
            if not subset:
                # fallback in case filtering unexpectedly resulted in no valid combinations
                subset = COMBOS[:]
            combo = random.choice(subset)
            # update rectangle colors
            possible_list = [i for i in range(10) if combo[i] == 1]
            self.update_possible_digits(possible_list)
            
            
            combo_str = ",".join(str(x) for x in combo)
            try:
                self.serial.send_line("COMBO")
                self.serial.send_line(combo_str)
            except Exception as e:
                self.log("Failed to send combo:", e)
                break
            self.log("Sent combo", combo_str)

            # collect trial data
            sample_rows = []  # list of (t_ms, [f0..f9])
            active_digit = None
            rand_delay = None
            collect_start = time.time()
            collecting = True
            max_collect_time = 127  # safety timeout in seconds
            while collecting and (time.time() - collect_start) < max_collect_time:
                # first check for status messages (ACTIVE_DIGIT, RAND_DELAY_MS, TRIAL_DONE)
                ln = self.serial.get_msg_nowait(timeout=0.01)
                if ln:
                    # log status messages
                    self.log("ESP ->", ln)
                    if ln.startswith("ACTIVE_DIGIT="):
                        try:
                            active_digit = int(ln.split("=", 1)[1])
                        except:
                            active_digit = None
                    elif ln.startswith("RAND_DELAY_MS="):
                        try:
                            rand_delay = int(ln.split("=", 1)[1])
                        except:
                            rand_delay = None
                    elif ln.strip() == "REST_START":
                        if len(sample_rows) == 6000:  # 60s at 100Hz
                            collecting = False
                            self.log("ESP reported REST_START and 6000 samples reached")
                            break
                        else:
                            self.log("ESP reported REST_START, ", len(sample_rows), " samples collected...")
                    elif ln.strip() == "TRIAL_DONE":
                        collecting = False
                        self.log("ESP reported TRIAL_DONE")
                        break
                    # ignore other messages
                # then drain sample queue without logging them
                try:
                    # batch read samples quickly
                    while True:
                        sample = self.serial.get_sample_nowait(timeout=0.001)
                        if sample is None:
                            break
                        tms, forces, active_digit, rand_delay = sample
                        sample_rows.append((tms, forces))
                except Exception:
                    pass
                time.sleep(0.0005)

            # finished collecting for this trial
            if len(sample_rows) == 0:
                self.log("No samples captured for trial (timeout or error).")
                if self.stop_requested:
                    break
                else:
                    continue

            # Save CSV
            self.trial_counter += 1
            ts_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            safe_user = (self.user_id.get().strip() or "user").replace(" ", "_")
            safe_sess = (self.session_id.get().strip() or "session").replace(" ", "_")
            fname = f"{safe_user}_{safe_sess}_trial{self.trial_counter}_{ts_str}.csv"
            outpath = os.path.join(OUT_DIR, fname)
            try:
                with open(outpath, "w", newline="") as f:
                    writer = csv.writer(f)
                    # Row 1: metadata single cell 
                    meta_str = f"UserID: {self.user_id.get().strip()}, SessionID: {self.session_id.get().strip()}, Date: {self.date_str.get().strip()}, Hand: {self.hand_choice.get().strip()}"
                    writer.writerow([meta_str])
                    # Row 2: random delay as single cell
                    writer.writerow([f"RandomDelay_ms: {rand_delay if rand_delay is not None else ''}"])
                    # Row 3: Combination aligned with force columns
                    writer.writerow(["Combination"] + list(combo))
                    # Row 4: Activated vector aligned with force columns
                    act_vec = [1 if (active_digit is not None and i == active_digit) else 0 for i in range(10)]
                    writer.writerow(["ActivatedDigit"] + act_vec)
                    # Row 5: header for samples
                    header = ["t_ms"] + [f"f{i}" for i in range(10)]
                    writer.writerow(header)
                    # Rows: samples
                    for (tms, forces) in sample_rows:
                        writer.writerow([tms] + forces)
                self.log(f"Saved trial {self.trial_counter} to {outpath} ({len(sample_rows)} samples)")
                # update GUI with combo and activated
                self.root.after(0, self._update_after_trial, combo, active_digit, rand_delay, outpath, len(sample_rows))
            except Exception as e:
                self.log("Failed to save CSV:", e)

            # short inter-trial pause
            time.sleep(0.1)

            if self.num_trials > 0 and self.trial_counter >= self.num_trials:
                self.log(f"Reached {self.num_trials} trials — stopping session.")
                self.stop_session()
                self.stop_requested = True

            if self.stop_requested:
                break

        # session finished
        self.session_active = False
        self.stop_requested = False
        self.status_var.set("Session stopped")
        self.log("Session ended")

    def _update_after_trial(self, combo, active_digit, rand_delay, outpath, n_samples):
        # update digit colors
        possible_list = [i for i in range(10) if combo[i] == 1]
        self.update_possible_digits(possible_list)
        info = (
            f"File: \n"
            f"{os.path.basename(outpath)}\n"
            f"Activated digit: {active_digit}\n"
            f"Random delay (ms): {rand_delay}\n"
            f"Samples: {n_samples}\n"
        )
        self.last_trial_info.set(info)
        self.status_var.set(f"Saved trial {self.trial_counter}")

    def _periodic_tasks(self):
        # read up to a few status messages and show them
        for _ in range(8):
            ln = self.serial.get_msg_nowait(timeout=0.001)
            if ln:
                # show only non-sample status lines
                self.log("ESP->", ln)
            else:
                break
        # refresh ports occasionally
        try:
            self._refresh_ports()
        except:
            pass
        # schedule next
        self.root.after(200, self._periodic_tasks)
    
    def open_results_window(self):
        """Launch a new window where the user can choose a CSV file and view plots."""
        ResultsWindow(self.root)

# for the results display window _____________________________________________________________________________________
class ResultsWindow:
    def __init__(self, master):
        self.top = tk.Toplevel(master)
        self.top.title("View Results")
        self.top.geometry("1200x900")
        self.top.configure(bg="white")

        # METADATA AREA
        self.meta_frame = tk.Frame(self.top, bg="white")
        self.meta_frame.pack(fill="x", padx=10, pady=(10, 2))

        title_lbl = tk.Label(
            self.meta_frame,
            text="Trial Data",
            font=("Arial", 14),
            bg="white"
        )
        title_lbl.grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 5))

        # Keys in desired order
        meta_keys = [
            "UserID", "SessionID", "Date",
            "Hand", "Random Delay (ms)",
            "Digit Combination", "Activated Digit", "Response Time (ms)"
        ]

        self.meta_labels = {}

        # 4-column compact grid
        for i, key in enumerate(meta_keys):
            r = 1 + (i // 4)   # starting row 1
            c = i % 4         # columns 0–2
            lbl = tk.Label(
                self.meta_frame,
                text=f"{key}: ",
                font=("Arial", 10),
                bg="white",
                anchor="w"
            )
            lbl.grid(row=r, column=c, sticky="w", padx=10)
            self.meta_labels[key] = lbl

        # FILE SELECT BUTTON 
        ttk.Button(self.top, text="Select CSV File", command=self.load_file).pack(pady=10)

        #  MATPLOTLIB FIGURE 
        self.fig = plt.Figure(figsize=(12, 8))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.top)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    # load file and plot
    def load_file(self):
        """Select CSV, load metadata + data, and plot 10 digit channels."""
        filepath = filedialog.askopenfilename(
            title="Select Trial CSV",
            filetypes=[("CSV Files", "*.csv")]
        )
        if not filepath:
            return

        # READ METADATA ROWS 
        try:
            with open(filepath, "r") as f:
                meta1 = f.readline().strip()      # Row 1
                meta2 = f.readline().strip()      # Row 2
                combo_row = f.readline().strip()  # Row 3
                act_row = f.readline().strip()    # Row 4
        except Exception as e:
            messagebox.showerror("Error", f"Failed to read metadata:\n{e}")
            return

        # Parse row 1 (UserID, SessionID, Date, Hand)
        meta_dict = {}
        for part in meta1.split(","):
            if ":" in part:
                k, v = part.split(":", 1)
                meta_dict[k.strip()] = v.strip()

        # Parse random delay
        try:
            rand_delay = meta2.split(":")[1].strip()
        except:
            rand_delay = ""

        # Parse digit combination and activated digit
        combo_vals = combo_row.split(",")[1:]
        act_vals = act_row.split(",")[1:]

        combo = list(map(int, combo_vals))
        act_vec = list(map(int, act_vals))

        activated_digit = act_vec.index(1) if 1 in act_vec else None

        # Update metadata labels in GUI
        self.meta_labels["UserID"].config(text=f"UserID: {meta_dict.get('UserID', '')}")
        self.meta_labels["SessionID"].config(text=f"SessionID: {meta_dict.get('SessionID', '')}")
        self.meta_labels["Date"].config(text=f"Date: {meta_dict.get('Date', '')}")
        self.meta_labels["Hand"].config(text=f"Hand: {meta_dict.get('Hand', '')}")
        self.meta_labels["Random Delay (ms)"].config(text=f"Random Delay (ms): {rand_delay}")
        self.meta_labels["Digit Combination"].config(text=f"Digit Combination: {combo}")
        self.meta_labels["Activated Digit"].config(text=f"Activated Digit: {activated_digit}")
        

        # IMPORT THE DATAFRAME
        try:
            df = pd.read_csv(filepath, skiprows=4)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load file:\n{e}")
            return

        # Extract numeric data
        t = df["t_ms"].to_numpy()
        forces = df[[f"f{i}" for i in range(10)]].to_numpy()

        # low pass filter to reduce high frequency vibration for response time detection
        filtered_forces = gaussian_filter(forces, sigma=2, mode='constant', cval=0)

        # process data for response times:
        # Compute baseline (first 1000 ms) - 1 sec of data, pre-vibration
        baseline_mask = t <= (t[0] + 1000)
        baseline_mean = forces[baseline_mask].mean(axis=0)
        baseline_std = forces[baseline_mask].std(axis=0)

        # Threshold = mean + 3*std - we can adjust this multiplier based on our force data
        thresh = baseline_mean + 3 * baseline_std

        # Compute response time for each digit
        response_times = []
        for i in range(10):
            exceed = np.where(filtered_forces[:, i] > thresh[i])[0] # look only after first second because responses should only occur after vibration starts; this also avoids false positives from baseline noise
            rt = t[exceed[0]] if len(exceed) > 0 and max(forces[exceed, i]) > 500 else None
            response_times.append(rt)

        response_time_active = response_times[activated_digit]- t[999] if activated_digit is not None and response_times[activated_digit] is not None else None
        self.meta_labels["Response Time (ms)"].config(text=f"Response Time (ms): {response_time_active if response_time_active is not None else 'N/A'}")

        # PLOT ALL 10 DIGIT CURVES
        self.fig.clear()

        for i in range(10):
            # Row (0–4)
            row = i if i < 5 else i - 5
            # Column (0 = left, 1 = right)
            col = 0 if i < 5 else 1

            # Convert row/col - subplot index (1–10)
            subplot_index = row * 2 + col + 1

            ax = self.fig.add_subplot(5, 2, subplot_index)

            ax.plot(t, forces[:, i], linewidth=1)
            ax.axhline(baseline_mean[i], linestyle="--", linewidth=1, color="gray")

            if response_times[i] is not None:
                ax.axvline(response_times[i], linestyle="--", color="blue", linewidth=1)
                

            if activated_digit is not None and i == activated_digit:
                ax.axvline(t[999], linestyle="--", color="green", linewidth=1)

            ax.set_title(f"Digit {i}")
            ax.set_xlabel("Time (ms)")
            ax.set_ylabel("Force")

        self.fig.tight_layout()
        self.canvas.draw()

# main _____________________________________________________________________________________________
def main():
    root = tk.Tk()
    app = AssessmentApp(root)

    def on_close():
        if app.session_active:
            if not messagebox.askyesno("Exit", "Session running. Exit anyway?"):
                return
            
        try:
            if app.serial and app.serial.ser and app.serial.ser.is_open:
                app.serial.send_line("GUI_CLOSED")
                print("Sent GUI_CLOSED")
        except Exception as e:
            print("Failed to send GUI_CLOSED:", e)  

        app.serial.close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()

if __name__ == "__main__":
    main()
