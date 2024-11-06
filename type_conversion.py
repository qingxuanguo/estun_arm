import struct

def byte_1_2_BOOL(byte_1):
    bytes_obj = bytes(byte_1)
    val = struct.unpack('<c', bytes_obj)[0]
    return val

def byte_1_2_SINT(byte_1):
    bytes_obj = bytes(byte_1)
    val = struct.unpack('<b', bytes_obj)[0]
    return val

def byte_1_2_USINT(byte_1):
    bytes_obj = bytes(byte_1)
    val = struct.unpack('<B', bytes_obj)[0]
    return val

def byte_1_2_BYTE(byte_1):
    bytes_obj = bytes(byte_1)
    val = struct.unpack('<B', bytes_obj)[0]
    return val

def byte_2_2_INT(byte_2):
    bytes_obj = bytes(byte_2)
    val = struct.unpack('<h', bytes_obj)[0]
    return val

def byte_2_2_UINT(byte_2):
    bytes_obj = bytes(byte_2)
    val = struct.unpack('<H', bytes_obj)[0]
    return val

def byte_4_2_DINT(byte_4):
    bytes_obj = bytes(byte_4)
    val = struct.unpack('<i', bytes_obj)[0]
    return val

def byte_4_2_UDINT(byte_4):
    bytes_obj = bytes(byte_4)
    val = struct.unpack('<I', bytes_obj)[0]
    return val

def byte_4_2_FLOAT(byte_4):
    bytes_obj = bytes(byte_4)
    val = struct.unpack('<f', bytes_obj)[0]
    return val

def byte_8_2_LINT(byte_8):
    bytes_obj = bytes(byte_8)
    val = struct.unpack('<q', bytes_obj)[0]
    return val

def byte_8_2_ULINT(byte_8):
    bytes_obj = bytes(byte_8)
    val = struct.unpack('<Q', bytes_obj)[0]
    return val

def byte_8_2_DOUBLE(byte_8):
    bytes_obj = bytes(byte_8)
    val = struct.unpack('<d', bytes_obj)[0]
    return val

def BOOL_2_byte_1(BOOL):
    byte_array = struct.pack("<c",BOOL)
    return byte_array
    
def SINT_2_byte_1(SINT):
    byte_array = struct.pack("<b",SINT)
    return byte_array
    
def USINT_2_byte_1(USINT):
    byte_array = struct.pack("<B",USINT)
    return byte_array
    
def BYTE_2_byte_1(BYTE):
    byte_array = struct.pack("<B",BYTE)
    return byte_array
    
def INT_2_byte_2(INT):
    byte_array = struct.pack("<h",INT)
    return byte_array
    
def UINT_2_byte_2(UINT):
    byte_array = struct.pack("<H",UINT)
    return byte_array
    
def DINT_2_byte_4(DINT):
    byte_array = struct.pack("<i",DINT)
    return byte_array
    
def UDINT_2_byte_4(UDINT):
    byte_array = struct.pack("<I",UDINT)
    return byte_array
    
def REAL_2_byte_4(REAL):
    byte_array = struct.pack("<f",REAL)
    return byte_array
    
def LINT_2_byte_8(LINT):
    byte_array = struct.pack("<q",LINT)
    return byte_array
    
def ULINT_2_byte_8(ULINT):
    byte_array = struct.pack("<Q",ULINT)
    return byte_array
    
def DOUBLE_2_byte_8(DOUBLE):
    byte_array = struct.pack("<d",DOUBLE)
    return byte_array