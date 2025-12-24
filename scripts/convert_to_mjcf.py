from robot_format_converter import FormatConverter
converter = FormatConverter()
schema = converter.convert('models/S1/urdf/humanoid_pkg.urdf', 'models/S1/urdf/humanoid_pkg-1.xml')