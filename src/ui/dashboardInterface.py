import os
import tkinter as tk
from typing import Any
import numpy as np
from PIL import Image, ImageTk
from interface.robot_control import RobotHandController


class FullscreenHandApp(tk.Toplevel):
    """Fullscreen 4-tile light-mode dashboard UI for HaptIQ.

    Uses the official HaptIQ x VEGA Hackathon palette:
      - Hackathon Yellow: #FFF201
      - Vega Yellow:      #FFDE01
      - Vega Secondary:   #444444
      - Derived Tertiary: #7B904B
      - Derived Fourth:   #B38CB4
      - Derived Fifth:    #DDF0FF
    """

    BASE_CANVAS_W = 700
    BASE_CANVAS_H = 900

    # Official HaptIQ Color Palette
    HACKATHON_YELLOW = "#FFF201"
    VEGA_YELLOW = "#FFDE01"
    VEGA_SECONDARY = "#444444"
    DERIVED_TERTIARY = "#7B904B"
    DERIVED_FOURTH = "#B38CB4"
    DERIVED_FIFTH = "#DDF0FF"

    # Light mode surfaces
    BG_MAIN = "#DDF0FF"
    BG_HEADER = "#FFFFFF"
    BG_TILE = "#FFFFFF"
    BORDER_TILE = "#444444"
    TEXT_PRIMARY = "#444444"
    TEXT_SECONDARY = "#444444"
    TEXT_MUTED = "#444444"
    HAND_CANVAS_BG = "#f2f1f6"

    def __init__(
        self,
        robot: RobotHandController,
        grasp_manager: Any = None,
        master: tk.Misc | None = None,
        companion_app: Any = None,
        fullscreen: bool = True,
    ):
        self._owns_hidden_root = False
        if master is None and tk._default_root is None:
            hidden_root = tk.Tk()
            hidden_root.withdraw()
            master = hidden_root
            self._owns_hidden_root = True

        super().__init__(master=master)
        self.robot = robot
        self.grasp_manager = grasp_manager
        self.companion_app = companion_app
        self.is_fullscreen = fullscreen

        self.pixel = 6
        self.current_slipping_contacts: list[str] = []

        self.offsets = {
            "Little": (150, 120),
            "Ring": (230, 110),
            "Middle": (310, 100),
            "Index": (390, 110),
            "Thumb": (420, 360),
            "Palm": (240, 520),
        }

        # State caching to avoid redundant Tk widget reconfigurations at 33 Hz
        self._last_state_key: str | None = None
        self._last_slip_key: tuple[bool, tuple[str, ...]] | None = None
        self._last_probe_key: tuple[str, str, float, float] | None = None

        # Scaled hand image & canvas item cache
        self._cached_img_size: tuple[int, int] = (0, 0)
        self._scale: float = 1.0
        self._pad_x: float = 0.0
        self._pad_y: float = 0.0
        self._rect_ids: dict[str, list[list[int]]] = {}
        self._rect_colors: dict[str, list[list[str]]] = {}
        self._oval_ids: dict[str, int] = {}
        self._oval_states: dict[str, tuple[int, int, int, bool, bool]] = {}

        self._load_base_images()
        self.setup_window()
        self.setup_ui()
        self._rebuild_canvas_items()

        # Initialize default tile visuals
        self.update_state_display("initialised")
        self.update_slip_display(False, [])
        self.update_probe_display("none", {})

    def _load_base_images(self) -> None:
        img_path = os.path.join(os.path.dirname(__file__), "hand.png")
        if not os.path.exists(img_path):
            img_path = os.path.join(os.getcwd(), "hand.png")
        self.base_hand_pil = Image.open(img_path)
        resized = self.base_hand_pil.resize(
            (self.BASE_CANVAS_W, self.BASE_CANVAS_H), Image.Resampling.LANCZOS
        )
        self.hand_img = ImageTk.PhotoImage(resized, master=self)
        self._cached_img_size = (self.BASE_CANVAS_W, self.BASE_CANVAS_H)

    def setup_window(self) -> None:
        self.title("HaptIQ — Tactile Intelligence Dashboard")
        self.configure(bg=self.BG_MAIN)

        sw = self.winfo_screenwidth()
        sh = self.winfo_screenheight()
        self.geometry(f"{sw}x{sh}+0+0")
        self.minsize(960, 640)

        if self.is_fullscreen:
            try:
                self.attributes("-fullscreen", True)
            except tk.TclError:
                pass

        self.bind("<Escape>", self._toggle_fullscreen)
        self.bind("<F11>", self._toggle_fullscreen)

    def _toggle_fullscreen(self, _event: Any = None) -> None:
        self.is_fullscreen = not self.is_fullscreen
        try:
            self.attributes("-fullscreen", self.is_fullscreen)
        except tk.TclError:
            pass

    def _create_tile(
        self, parent: tk.Misc, title: str, accent_color: str = VEGA_YELLOW
    ) -> tuple[tk.Frame, tk.Frame, tk.Frame, tk.Label]:
        """Creates a styled light-mode dashboard tile with a top accent bar and header."""
        outer = tk.Frame(
            parent,
            bg=self.BORDER_TILE,
            padx=2,
            pady=2,
        )
        accent_bar = tk.Frame(outer, bg=accent_color, height=8)
        accent_bar.pack(side="top", fill="x")

        inner = tk.Frame(outer, bg=self.BG_TILE, padx=24, pady=18)
        inner.pack(side="top", fill="both", expand=True)

        header_frame = tk.Frame(inner, bg=self.BG_TILE)
        header_frame.pack(side="top", fill="x")

        title_lbl = tk.Label(
            header_frame,
            text=title,
            font=("Helvetica", 13, "bold"),
            fg=self.VEGA_SECONDARY,
            bg=self.BG_TILE,
            anchor="w",
        )
        title_lbl.pack(side="left")

        return outer, inner, accent_bar, title_lbl

    def setup_ui(self) -> None:
        # ── Top Bar (Branding + Single "Open" Button) ──
        self.top_bar = tk.Frame(
            self,
            bg=self.BG_HEADER,
            padx=28,
            pady=14,
            highlightbackground=self.VEGA_SECONDARY,
            highlightthickness=2,
        )
        self.top_bar.pack(side="top", fill="x")

        brand_frame = tk.Frame(self.top_bar, bg=self.BG_HEADER)
        brand_frame.pack(side="left")

        brand_badge = tk.Frame(
            brand_frame,
            bg=self.VEGA_YELLOW,
            padx=12,
            pady=4,
            highlightbackground=self.VEGA_SECONDARY,
            highlightthickness=2,
        )
        brand_badge.pack(side="left")
        tk.Label(
            brand_badge,
            text="HAPTIQ",
            font=("Helvetica", 20, "bold"),
            fg=self.VEGA_SECONDARY,
            bg=self.VEGA_YELLOW,
        ).pack(side="left")

        tk.Label(
            brand_frame,
            text="   TACTILE INTELLIGENCE MONITOR",
            font=("Helvetica", 13, "bold"),
            fg=self.VEGA_SECONDARY,
            bg=self.BG_HEADER,
        ).pack(side="left", padx=(6, 0))

        # Single "Open" button (custom Frame+Label so macOS Aqua respects colors)
        self.open_btn_border = tk.Frame(
            self.top_bar,
            bg=self.VEGA_SECONDARY,
            padx=2,
            pady=2,
            cursor="hand2",
        )
        self.open_btn_border.pack(side="right")

        self.open_button = tk.Label(
            self.open_btn_border,
            text="OPEN",
            font=("Helvetica", 14, "bold"),
            fg=self.VEGA_SECONDARY,
            bg=self.VEGA_YELLOW,
            padx=28,
            pady=8,
            cursor="hand2",
        )
        self.open_button.pack()

        for widget in (self.open_btn_border, self.open_button):
            widget.bind("<Button-1>", lambda _e: self.open_hand())
            widget.bind(
                "<Enter>",
                lambda _e: self.open_button.config(bg=self.HACKATHON_YELLOW),
            )
            widget.bind(
                "<Leave>",
                lambda _e: self.open_button.config(bg=self.VEGA_YELLOW),
            )

        # ── Main 4-Tile Bento Grid ──
        self.grid_container = tk.Frame(self, bg=self.BG_MAIN, padx=24, pady=20)
        self.grid_container.pack(side="top", fill="both", expand=True)

        self.grid_container.grid_columnconfigure(0, weight=5, uniform="col")
        self.grid_container.grid_columnconfigure(1, weight=6, uniform="col")
        self.grid_container.grid_rowconfigure(0, weight=1, uniform="row")
        self.grid_container.grid_rowconfigure(1, weight=1, uniform="row")
        self.grid_container.grid_rowconfigure(2, weight=1, uniform="row")

        # ── TILE 1: STATE ──
        (
            self.state_tile_outer,
            self.state_tile,
            self.state_accent,
            self.state_header_lbl,
        ) = self._create_tile(self.grid_container, "STATE", self.VEGA_YELLOW)
        self.state_tile_outer.grid(
            row=0, column=0, sticky="nsew", padx=(0, 12), pady=(0, 12)
        )

        state_body = tk.Frame(self.state_tile, bg=self.BG_TILE)
        state_body.pack(side="top", fill="both", expand=True)

        self.state_label = tk.Label(
            state_body,
            text="INITIALISED",
            font=("Helvetica", 36, "bold"),
            fg=self.VEGA_SECONDARY,
            bg=self.BG_TILE,
            anchor="w",
        )
        self.state_label.pack(side="top", fill="x", pady=(18, 6))

        self.state_sub_label = tk.Label(
            state_body,
            text="System initialized and ready",
            font=("Helvetica", 14),
            fg=self.VEGA_SECONDARY,
            bg=self.BG_TILE,
            anchor="w",
        )
        self.state_sub_label.pack(side="top", fill="x")
        self._state_body = state_body

        # ── TILE 2: SLIPPAGE DETECTION ──
        (
            self.slip_tile_outer,
            self.slip_tile,
            self.slip_accent,
            self.slip_header_lbl,
        ) = self._create_tile(
            self.grid_container, "SLIPPAGE DETECTION", self.DERIVED_TERTIARY
        )
        self.slip_tile_outer.grid(
            row=1, column=0, sticky="nsew", padx=(0, 12), pady=12
        )

        slip_body = tk.Frame(self.slip_tile, bg=self.BG_TILE)
        slip_body.pack(side="top", fill="both", expand=True)

        self.slip_banner = tk.Label(
            slip_body,
            text="GRIP: STABLE",
            font=("Helvetica", 36, "bold"),
            fg=self.DERIVED_TERTIARY,
            bg=self.BG_TILE,
            anchor="w",
        )
        self.slip_banner.pack(side="top", fill="x", pady=(18, 6))

        self.slip_sub_label = tk.Label(
            slip_body,
            text="No tangential slip or force drop detected",
            font=("Helvetica", 14),
            fg=self.VEGA_SECONDARY,
            bg=self.BG_TILE,
            anchor="w",
            wraplength=520,
            justify="left",
        )
        self.slip_sub_label.pack(side="top", fill="x")
        self._slip_body = slip_body

        # ── TILE 3: SENSOR DETECTED ──
        (
            self.probe_tile_outer,
            self.probe_tile,
            self.probe_accent,
            self.probe_header_lbl,
        ) = self._create_tile(
            self.grid_container, "SENSOR DETECTED", self.DERIVED_FOURTH
        )
        self.probe_tile_outer.grid(
            row=2, column=0, sticky="nsew", padx=(0, 12), pady=(12, 0)
        )

        probe_body = tk.Frame(self.probe_tile, bg=self.BG_TILE)
        probe_body.pack(side="top", fill="both", expand=True)

        self.probe_banner = tk.Label(
            probe_body,
            text="NOT PROBED",
            font=("Helvetica", 36, "bold"),
            fg=self.VEGA_SECONDARY,
            bg=self.BG_TILE,
            anchor="w",
        )
        self.probe_banner.pack(side="top", fill="x", pady=(18, 6))

        self.probe_sub_label = tk.Label(
            probe_body,
            text="Awaiting confirmed grasp for active thumb identification",
            font=("Helvetica", 14),
            fg=self.VEGA_SECONDARY,
            bg=self.BG_TILE,
            anchor="w",
        )
        self.probe_sub_label.pack(side="top", fill="x")
        self._probe_body = probe_body

        # ── TILE 4: TACTILE FEEDBACK (Spans all 3 rows on the right) ──
        (
            self.tactile_tile_outer,
            self.tactile_tile,
            self.tactile_accent,
            self.tactile_header_lbl,
        ) = self._create_tile(
            self.grid_container, "TACTILE FEEDBACK", self.VEGA_YELLOW
        )
        self.tactile_tile_outer.grid(
            row=0, column=1, rowspan=3, sticky="nsew", padx=(12, 0), pady=0
        )

        # Compact legend in the tactile tile header
        legend_frame = tk.Frame(self.tactile_header_lbl.master, bg=self.BG_TILE)
        legend_frame.pack(side="right")
        for label_txt, col in (
            ("Low", "#00aa00"),
            ("Med", "#cccc00"),
            ("High", "#ff8800"),
            ("Peak", "#ff0000"),
            ("CoP", "#0055ff"),
            ("Slip", "#ff2222"),
        ):
            swatch = tk.Frame(legend_frame, bg=col, width=10, height=10)
            swatch.pack(side="left", padx=(8, 3))
            tk.Label(
                legend_frame,
                text=label_txt,
                font=("Helvetica", 10),
                fg=self.VEGA_SECONDARY,
                bg=self.BG_TILE,
            ).pack(side="left")

        self.canvas_container = tk.Frame(
            self.tactile_tile,
            bg=self.HAND_CANVAS_BG,
            highlightbackground=self.VEGA_SECONDARY,
            highlightthickness=1,
        )
        self.canvas_container.pack(
            side="top", fill="both", expand=True, pady=(14, 0)
        )

        self.canvas = tk.Canvas(
            self.canvas_container,
            bg=self.HAND_CANVAS_BG,
            highlightthickness=0,
        )
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", self._on_canvas_configure)

    def _rebuild_canvas_items(self) -> None:
        """Pre-allocates canvas image, 1,062 taxel rectangles, and 17 CoP ovals."""
        if not self.winfo_exists():
            return
        self.canvas.delete("all")
        self._rect_ids.clear()
        self._rect_colors.clear()
        self._oval_ids.clear()
        self._oval_states.clear()

        scale = self._scale
        pad_x = self._pad_x
        pad_y = self._pad_y
        px = self.pixel * scale

        cx_img = pad_x + (self.BASE_CANVAS_W * scale) / 2.0
        cy_img = pad_y + (self.BASE_CANVAS_H * scale) / 2.0
        self.canvas.create_image(cx_img, cy_img, image=self.hand_img)

        default_color = "#00aa00"
        for name, _addr, h, w, key, rel in self.robot.sensor_groups:
            x0 = pad_x + (self.offsets[key][0] + rel[0]) * scale
            y0 = pad_y + (self.offsets[key][1] + rel[1]) * scale

            rows_ids: list[list[int]] = []
            rows_cols: list[list[str]] = []
            for i in range(h):
                r_ids: list[int] = []
                r_cols: list[str] = []
                for j in range(w):
                    rid = self.canvas.create_rectangle(
                        x0 + j * px,
                        y0 + i * px,
                        x0 + (j + 1) * px,
                        y0 + (i + 1) * px,
                        fill=default_color,
                        width=0,
                        stipple="gray25",
                    )
                    r_ids.append(rid)
                    r_cols.append(default_color)
                rows_ids.append(r_ids)
                rows_cols.append(r_cols)

            self._rect_ids[name] = rows_ids
            self._rect_colors[name] = rows_cols
            self._oval_ids[name] = self.canvas.create_oval(
                0, 0, 0, 0, fill="#0055ff", state="hidden"
            )
            self._oval_states[name] = (0, 0, 0, False, False)

    def _on_canvas_configure(self, event: Any) -> None:
        cw = max(1, int(event.width))
        ch = max(1, int(event.height))
        scale = min(cw / self.BASE_CANVAS_W, ch / self.BASE_CANVAS_H)
        target_w = max(1, int(self.BASE_CANVAS_W * scale))
        target_h = max(1, int(self.BASE_CANVAS_H * scale))

        if (target_w, target_h) == self._cached_img_size and abs(scale - self._scale) < 1e-4:
            return

        self._scale = scale
        self._pad_x = (cw - target_w) / 2.0
        self._pad_y = (ch - target_h) / 2.0

        resized = self.base_hand_pil.resize(
            (target_w, target_h), Image.Resampling.LANCZOS
        )
        self.hand_img = ImageTk.PhotoImage(resized, master=self)
        self._cached_img_size = (target_w, target_h)

        self._rebuild_canvas_items()

    def open_hand(self) -> None:
        if self.grasp_manager is not None:
            self.grasp_manager.reset(self.robot)
        else:
            self.robot.open_hand()
        self.update_state_display("open")
        self.update_slip_display(False, [])
        self.update_probe_display("none", {})

        # Keep companion HandApp synchronized if running alongside
        if self.companion_app is not None:
            try:
                if hasattr(self.companion_app, "sliders"):
                    for s in self.companion_app.sliders:
                        s.set(1000)
                if hasattr(self.companion_app, "update_slip_display"):
                    self.companion_app.update_slip_display(False, [])
                if hasattr(self.companion_app, "update_probe_display"):
                    self.companion_app.update_probe_display("none", {})
                if hasattr(self.companion_app, "update_state_display"):
                    self.companion_app.update_state_display("open")
            except tk.TclError:
                pass

    def draw(self) -> None:
        if not self.winfo_exists():
            return
        if not self._rect_ids:
            self._rebuild_canvas_items()

        scale = self._scale
        pad_x = self._pad_x
        pad_y = self._pad_y
        px = self.pixel * scale
        slipping_set = set(getattr(self, "current_slipping_contacts", []))

        for name, addr, h, w, key, rel in self.robot.sensor_groups:
            if name in self.robot.last_sensor_data:
                raw = self.robot.last_sensor_data[name]
            else:
                data = self.robot.safe_read(addr, h * w)
                if not data:
                    continue
                raw = np.array(data)

            offset = self.robot.sensor_offsets.get(name, np.zeros(len(raw)))
            matrix = (raw - offset).reshape(h, w)
            matrix[matrix < 0] = 0

            rect_grid = self._rect_ids.get(name)
            color_grid = self._rect_colors.get(name)
            if rect_grid is not None and color_grid is not None:
                for i in range(h):
                    r_ids = rect_grid[i]
                    r_cols = color_grid[i]
                    row_vals = matrix[i]
                    for j in range(w):
                        v = row_vals[j] / 4000.0
                        if v < 0.2:
                            color = "#00aa00"
                        elif v < 0.5:
                            color = "#cccc00"
                        elif v < 0.8:
                            color = "#ff8800"
                        else:
                            color = "#ff0000"

                        if r_cols[j] != color:
                            r_cols[j] = color
                            self.canvas.itemconfig(r_ids[j], fill=color)

            oval_id = self._oval_ids.get(name)
            if oval_id is None:
                continue

            active = matrix > 50
            if np.sum(active) < 2:
                prev_state = self._oval_states.get(name)
                if prev_state is None or prev_state[4]:
                    self.canvas.itemconfig(oval_id, state="hidden")
                    self._oval_states[name] = (0, 0, 0, False, False)
                continue

            x0 = pad_x + (self.offsets[key][0] + rel[0]) * scale
            y0 = pad_y + (self.offsets[key][1] + rel[1]) * scale

            y_idx, x_idx = np.where(active)
            cx = int(np.mean(x_idx))
            cy = int(np.mean(y_idx))
            force = np.mean(matrix[active])
            size = int((3 + int(force / 200)) * scale)

            is_slipping = name in slipping_set
            new_oval_state = (cx, cy, size, is_slipping, True)
            if self._oval_states.get(name) != new_oval_state:
                self._oval_states[name] = new_oval_state
                circle_color = "#ff2222" if is_slipping else "#0055ff"
                outline_color = "#ffcc00" if is_slipping else ""

                self.canvas.coords(
                    oval_id,
                    x0 + cx * px,
                    y0 + cy * px,
                    x0 + cx * px + size,
                    y0 + cy * px + size,
                )
                self.canvas.itemconfig(
                    oval_id,
                    state="normal",
                    fill=circle_color,
                    outline=outline_color,
                    width=2 if is_slipping else 0,
                )

    def update_state_display(self, state: str) -> None:
        if not hasattr(self, "state_label"):
            return
        state_lower = (state or "initialised").lower()
        if state_lower == self._last_state_key:
            return
        self._last_state_key = state_lower

        # Map each state to the 6-color palette:
        # (display_text, top_accent_color, fg_color, tile_bg_color, subtitle)
        state_meta = {
            "starting": (
                "STARTING",
                self.VEGA_SECONDARY,
                self.VEGA_SECONDARY,
                self.BG_TILE,
                "Initializing controller & Modbus interface",
            ),
            "initialised": (
                "INITIALISED",
                self.VEGA_SECONDARY,
                self.VEGA_SECONDARY,
                self.BG_TILE,
                "System initialized and ready",
            ),
            "open": (
                "OPEN",
                self.VEGA_YELLOW,
                self.VEGA_SECONDARY,
                self.BG_TILE,
                "Hand open — awaiting tactile contact trigger",
            ),
            "closing": (
                "CLOSING",
                self.VEGA_YELLOW,
                self.VEGA_SECONDARY,
                self.HACKATHON_YELLOW,
                "Contact detected — closing fingers onto object",
            ),
            "settling": (
                "SETTLING",
                self.VEGA_SECONDARY,
                self.VEGA_SECONDARY,
                self.VEGA_YELLOW,
                "Verifying cylindrical grip geometry & force balance",
            ),
            "steady_state": (
                "STEADY STATE",
                self.DERIVED_TERTIARY,
                "#FFFFFF",
                self.DERIVED_TERTIARY,
                "Stable grasp maintained",
            ),
            "cylinder_confirmed": (
                "CYLINDER CONFIRMED",
                self.DERIVED_TERTIARY,
                "#FFFFFF",
                self.DERIVED_TERTIARY,
                "Cylindrical grasp verified and locked",
            ),
            "thumb_probing": (
                "THUMB PROBING",
                self.DERIVED_FOURTH,
                self.VEGA_SECONDARY,
                self.DERIVED_FOURTH,
                "Executing 3-station active thumb scan",
            ),
            "probing_complete": (
                "PROBING COMPLETE",
                self.DERIVED_TERTIARY,
                self.VEGA_SECONDARY,
                self.DERIVED_FIFTH,
                "Active thumb scan completed",
            ),
            "resetting": (
                "RESETTING",
                self.VEGA_SECONDARY,
                self.VEGA_SECONDARY,
                self.HACKATHON_YELLOW,
                "Releasing object and re-arming tactile triggers",
            ),
        }
        display_text, accent, fg_col, bg_col, subtitle = state_meta.get(
            state_lower,
            (
                state.upper().replace("_", " "),
                self.VEGA_YELLOW,
                self.VEGA_SECONDARY,
                self.BG_TILE,
                "Active state",
            ),
        )

        self.state_accent.config(bg=accent)
        self.state_tile.config(bg=bg_col)
        self._state_body.config(bg=bg_col)
        self.state_header_lbl.master.config(bg=bg_col)
        self.state_header_lbl.config(bg=bg_col, fg=fg_col)
        self.state_label.config(text=display_text, fg=fg_col, bg=bg_col)
        self.state_sub_label.config(text=subtitle, fg=fg_col, bg=bg_col)
        self.title(f"HaptIQ Dashboard — [{display_text}]")

    def update_slip_display(
        self, slip_detected: bool, slipping_contacts: list[str]
    ) -> None:
        if not hasattr(self, "slip_banner"):
            return
        self.current_slipping_contacts = slipping_contacts
        slip_key = (bool(slip_detected), tuple(slipping_contacts))
        if slip_key == self._last_slip_key:
            return
        self._last_slip_key = slip_key

        if slip_detected and slipping_contacts:
            contact_str = ", ".join(slipping_contacts)
            alert_bg = self.HACKATHON_YELLOW
            alert_accent = self.VEGA_SECONDARY
            self.slip_accent.config(bg=alert_accent)
            self.slip_tile.config(bg=alert_bg)
            self._slip_body.config(bg=alert_bg)
            self.slip_header_lbl.master.config(bg=alert_bg)
            self.slip_header_lbl.config(bg=alert_bg, fg=self.VEGA_SECONDARY)
            self.slip_banner.config(
                text="SLIP DETECTED",
                fg=self.VEGA_SECONDARY,
                bg=alert_bg,
            )
            self.slip_sub_label.config(
                text=f"Slipping contacts: {contact_str}",
                fg=self.VEGA_SECONDARY,
                bg=alert_bg,
            )
        else:
            stable_bg = self.BG_TILE
            stable_accent = self.DERIVED_TERTIARY
            self.slip_accent.config(bg=stable_accent)
            self.slip_tile.config(bg=stable_bg)
            self._slip_body.config(bg=stable_bg)
            self.slip_header_lbl.master.config(bg=stable_bg)
            self.slip_header_lbl.config(bg=stable_bg, fg=self.VEGA_SECONDARY)
            self.slip_banner.config(
                text="GRIP: STABLE",
                fg=self.DERIVED_TERTIARY,
                bg=stable_bg,
            )
            self.slip_sub_label.config(
                text="No tangential slip or force drop detected",
                fg=self.VEGA_SECONDARY,
                bg=stable_bg,
            )

    def update_probe_display(
        self,
        probe_state: str,
        probe_summary: dict | None = None,
        state: str | None = None,
    ) -> None:
        if not hasattr(self, "probe_banner"):
            return
        summary = probe_summary or {}
        state_lower = (state or "").lower()
        mean_bend = float(summary.get("mean_bend", 0.0))
        mean_force = float(summary.get("mean_force", 0.0))

        probe_key = (
            probe_state or "none",
            state_lower,
            round(mean_bend, 0),
            round(mean_force, 0),
        )
        if probe_key == self._last_probe_key:
            return
        self._last_probe_key = probe_key

        if state_lower == "thumb_probing" or probe_state == "probing":
            bg_col = self.DERIVED_FOURTH
            accent = self.VEGA_SECONDARY
            self.probe_accent.config(bg=accent)
            self.probe_tile.config(bg=bg_col)
            self._probe_body.config(bg=bg_col)
            self.probe_header_lbl.master.config(bg=bg_col)
            self.probe_header_lbl.config(bg=bg_col, fg=self.VEGA_SECONDARY)
            self.probe_banner.config(
                text="PROBING THUMB...",
                fg=self.VEGA_SECONDARY,
                bg=bg_col,
            )
            self.probe_sub_label.config(
                text="Sweeping thumb across upper housing stations...",
                fg=self.VEGA_SECONDARY,
                bg=bg_col,
            )
        elif probe_state == "VEGABAR 38":
            bg_col = self.VEGA_YELLOW
            accent = self.DERIVED_TERTIARY
            self.probe_accent.config(bg=accent)
            self.probe_tile.config(bg=bg_col)
            self._probe_body.config(bg=bg_col)
            self.probe_header_lbl.master.config(bg=bg_col)
            self.probe_header_lbl.config(bg=bg_col, fg=self.VEGA_SECONDARY)
            self.probe_banner.config(
                text="VEGABAR 38",
                fg=self.VEGA_SECONDARY,
                bg=bg_col,
            )
            self.probe_sub_label.config(
                text=f"Mean Bend: {mean_bend:.0f}   •   Mean Force: {mean_force:.0f} g",
                fg=self.VEGA_SECONDARY,
                bg=bg_col,
            )
        elif probe_state == "VEGABAR 29":
            bg_col = self.DERIVED_FIFTH
            accent = self.DERIVED_FOURTH
            self.probe_accent.config(bg=accent)
            self.probe_tile.config(bg=bg_col)
            self._probe_body.config(bg=bg_col)
            self.probe_header_lbl.master.config(bg=bg_col)
            self.probe_header_lbl.config(bg=bg_col, fg=self.VEGA_SECONDARY)
            self.probe_banner.config(
                text="VEGABAR 29",
                fg=self.VEGA_SECONDARY,
                bg=bg_col,
            )
            self.probe_sub_label.config(
                text=f"Mean Bend: {mean_bend:.0f}   •   Mean Force: {mean_force:.0f} g",
                fg=self.VEGA_SECONDARY,
                bg=bg_col,
            )
        else:
            accent = self.DERIVED_FOURTH
            self.probe_accent.config(bg=accent)
            self.probe_tile.config(bg=self.BG_TILE)
            self._probe_body.config(bg=self.BG_TILE)
            self.probe_header_lbl.master.config(bg=self.BG_TILE)
            self.probe_header_lbl.config(bg=self.BG_TILE, fg=self.VEGA_SECONDARY)
            self.probe_banner.config(
                text="NOT PROBED",
                fg=self.VEGA_SECONDARY,
                bg=self.BG_TILE,
            )
            self.probe_sub_label.config(
                text="Awaiting confirmed grasp for active thumb identification",
                fg=self.VEGA_SECONDARY,
                bg=self.BG_TILE,
            )

    def main(self, state: str | None = None, data: Any = None) -> None:
        if not self.winfo_exists():
            return
        if data is not None:
            if state is None:
                state = data.get("state")
            self.update_slip_display(
                data.get("slip_detected", False),
                data.get("slipping_contacts", []),
            )
            self.update_probe_display(
                data.get("probe_state", "none"),
                data.get("probe_summary", {}),
                state=state,
            )
        if state:
            self.update_state_display(state)
        self.draw()
        try:
            self.update_idletasks()
            self.update()
        except tk.TclError:
            pass

    def destroy(self) -> None:
        master = self.master
        owns_root = getattr(self, "_owns_hidden_root", False)
        try:
            super().destroy()
        except tk.TclError:
            pass
        if owns_root and master is not None:
            try:
                master.destroy()
            except tk.TclError:
                pass
