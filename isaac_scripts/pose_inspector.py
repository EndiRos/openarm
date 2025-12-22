import omni.ui as ui
import omni.usd
from pxr import Usd, UsdGeom, Gf
import math

# Identificador único para la ventana para evitar duplicados
WINDOW_TITLE = "OpenArm Pose Inspector"

class PoseInspectorWindow(ui.Window):
    def __init__(self, title: str, **kwargs):
        super().__init__(title, width=400, height=500, **kwargs)
        self.frame.set_build_fn(self._build_fn)
        self._target_prim_path = None
        self._ref_prim_path = None
        
    def _build_fn(self):
        with ui.VStack(spacing=10, padding=15):
            # Header
            with ui.HStack(height=30):
                ui.Label("📍 Pose Inspector", style={"font_size": 20, "font_weight": "bold"})
            
            ui.Separator(height=10)

            # SECCIÓN 1: Selección
            with ui.CollapsableFrame("Selection", height=0, collapsed=False):
                with ui.VStack(spacing=8, padding=5):
                    # Target
                    with ui.HStack(height=25):
                        ui.Label("Target:", width=60, style={"color": 0xFFDDDDDD})
                        self._target_label = ui.StringField(read_only=True, style={"color": 0xFF88AAFF})
                        ui.Spacer(width=5)
                        ui.Button("Get", width=40, clicked_fn=self._on_get_target, tooltip="Get selected object from Stage")

                    # Reference
                    with ui.HStack(height=25):
                        ui.Label("Ref:", width=60, style={"color": 0xFFDDDDDD})
                        self._ref_label = ui.StringField(read_only=True, style={"color": 0xFFAAAAAA})
                        ui.Spacer(width=5)
                        ui.Button("Get", width=40, clicked_fn=self._on_get_ref, tooltip="Get reference frame")
                        ui.Button("X", width=25, clicked_fn=self._on_clear_ref, tooltip="Clear reference (Use World)")

            # SECCIÓN 2: Acción
            ui.Spacer(height=5)
            ui.Button("CALCULATE POSE", height=40, clicked_fn=self._on_calculate, 
                     style={"background_color": 0xFF228822, "font_size": 16, "border_radius": 5})
            ui.Spacer(height=5)

            # SECCIÓN 3: Resultados
            with ui.CollapsableFrame("Results", height=0, collapsed=False):
                with ui.VStack(spacing=8, padding=5):
                    self._create_result_field("Position (XYZ)", "0.0, 0.0, 0.0")
                    self._pos_field = self._last_created_field
                    
                    self._create_result_field("Quaternion (XYZW)", "0.0, 0.0, 0.0, 1.0")
                    self._rot_field = self._last_created_field
                    
                    self._create_result_field("Euler RPY (Deg)", "0.0, 0.0, 0.0")
                    self._rpy_field = self._last_created_field

            # SECCIÓN 4: Código C++
            ui.Label("C++ Code (Copy & Paste):", height=20, style={"color": 0xFF88AAFF, "margin_top": 10})
            with ui.ScrollingFrame(height=80, style={"background_color": 0xFF222222, "border_radius": 5}):
                self._cpp_field = ui.StringField(read_only=False, multiline=True)
                self._cpp_field.model.set_value("// Select objects and click Calculate")

    def _create_result_field(self, label, default_val):
        with ui.VStack(height=0, spacing=2):
            ui.Label(label, style={"font_size": 12, "color": 0xFFAAAAAA})
            self._last_created_field = ui.StringField(read_only=True)
            self._last_created_field.model.set_value(default_val)

    def _get_current_selection(self):
        ctx = omni.usd.get_context()
        selection = ctx.get_selection().get_selected_prim_paths()
        if selection:
            return selection[0]
        return None

    def _on_get_target(self):
        path = self._get_current_selection()
        if path:
            self._target_prim_path = path
            self._target_label.model.set_value(path)
        else:
            self._target_label.model.set_value("Select an object first")

    def _on_get_ref(self):
        path = self._get_current_selection()
        if path:
            self._ref_prim_path = path
            self._ref_label.model.set_value(path)
        else:
            self._ref_label.model.set_value("Select an object first")

    def _on_clear_ref(self):
        self._ref_prim_path = None
        self._ref_label.model.set_value("World (Default)")

    def _on_calculate(self):
        if not self._target_prim_path:
            self._pos_field.model.set_value("ERROR: No target selected")
            return

        stage = omni.usd.get_context().get_stage()
        target_prim = stage.GetPrimAtPath(self._target_prim_path)
        
        if not target_prim.IsValid():
            self._pos_field.model.set_value("ERROR: Invalid target")
            return

        # Get World Transform of Target
        target_xform = UsdGeom.Xformable(target_prim)
        time_code = Usd.TimeCode.Default()
        target_world_transform = target_xform.ComputeLocalToWorldTransform(time_code)

        final_transform = target_world_transform

        # If reference is set, compute relative transform
        if self._ref_prim_path:
            ref_prim = stage.GetPrimAtPath(self._ref_prim_path)
            if ref_prim.IsValid():
                ref_xform = UsdGeom.Xformable(ref_prim)
                ref_world_transform = ref_xform.ComputeLocalToWorldTransform(time_code)
                # T_rel = T_ref_inv * T_target
                final_transform = target_world_transform * ref_world_transform.GetInverse()

        # Extract Translation
        translation = final_transform.ExtractTranslation()
        
        # Extract Rotation (Quaternion)
        rotation_quat = final_transform.ExtractRotationQuat()
        real = rotation_quat.GetReal()
        imag = rotation_quat.GetImaginary()
        
        # Extract Euler
        rot_obj = Gf.Rotation(rotation_quat)
        euler = rot_obj.Decompose(Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis())
        
        # Update UI
        self._pos_field.model.set_value(f"{translation[0]:.5f}, {translation[1]:.5f}, {translation[2]:.5f}")
        self._rot_field.model.set_value(f"{imag[0]:.5f}, {imag[1]:.5f}, {imag[2]:.5f}, {real:.5f}")
        self._rpy_field.model.set_value(f"{euler[0]:.2f}, {euler[1]:.2f}, {euler[2]:.2f}")
        
        # C++ Snippet
        cpp_code = f"createpose({{{translation[0]:.5f}, {translation[1]:.5f}, {translation[2]:.5f}}}, {{{imag[0]:.5f}, {imag[1]:.5f}, {imag[2]:.5f}, {real:.5f}}});"
        self._cpp_field.model.set_value(cpp_code)

# --- Lógica de Inicialización ---

# 1. Cerrar ventana existente si la hay (para recargar el script sin duplicar)
if "_pose_inspector_window" in globals() and _pose_inspector_window:
    _pose_inspector_window.destroy()
    _pose_inspector_window = None

# 2. Crear nueva instancia
_pose_inspector_window = PoseInspectorWindow(WINDOW_TITLE)

# 3. Mostrar y anclar (Docking)
# Intentamos anclarla a la derecha, junto a "Property" si existe, o flotante.
_pose_inspector_window.show()
# Nota: El docking automático complejo requiere acceso a la estructura interna del layout, 
# pero .show() la mostrará flotante y el usuario puede arrastrarla donde quiera.
