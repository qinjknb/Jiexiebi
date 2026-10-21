
def comports():
    class P:
        def __init__(self, d, desc):
            self.device = d
            self.description = desc
    return [P("COM3", "USB-SERIAL CH340 (COM3)")]
