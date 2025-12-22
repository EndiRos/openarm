# Isaac Sim Pose Inspector (Script)

Este es un script de Python independiente que crea un panel flotante en Isaac Sim. No requiere instalación de extensiones.

## Cómo usarlo

1.  Abre **Isaac Sim**.
2.  Ve al menú **Window** -> **Script Editor**.
3.  Abre el archivo `isaac_scripts/pose_inspector.py` o copia su contenido.
4.  Presiona **Run** (o `Ctrl+Enter`).
5.  Aparecerá la ventana "Pose Inspector". Puedes arrastrarla y acoplarla (dock) donde quieras en la interfaz.

## Funcionalidades

*   **Target**: Selecciona un objeto y pulsa "Get" para definirlo.
*   **Ref**: (Opcional) Selecciona un objeto de referencia.
*   **Calculate**: Obtiene la pose.
*   **C++ Code**: Genera el código para copiar y pegar.

## Nota
Si cierras Isaac Sim, tendrás que volver a ejecutar el script. Si quieres que sea permanente, la forma oficial es crear una Extensión, pero este script es la forma más rápida y sencilla de tener la herramienta "ya".