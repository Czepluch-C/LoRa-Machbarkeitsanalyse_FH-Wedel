"""
Created by Christian Czepluch
"""


from Crypto.Random import get_random_bytes
from Crypto.Cipher import AES
from Crypto.Util.Padding import pad, unpad

import LoRa_Packet

# intValues need to be under 255


def IntArr_To_ByteArr(intArr):
    """
    length = len(intArr)
    byteArr = bytearray()
    for i in range(length):
        try:
            byteArr.append(intArr[i])
        except:
            return None
    return byteArr
    """
    return bytearray(intArr)


def ByteArr_ToIntArr(byteArr):
    """
    iArr = []
    for i in range(len(byteArr)):
        iArr.append(byteArr[i])
    return iArr
    """
    return list(byteArr)


class decipherC:
    """
    class to decipher LoRa Packets with AES
    """

    def __init__(self, key, lorapacket):
        self.key = key

        if lorapacket.isRightLength:
            try:
                __iv = IntArr_To_ByteArr(lorapacket.IV_int)
                __cipheredData = IntArr_To_ByteArr(lorapacket.data_int)
            except:
                raise ValueError("Couldn't convert IV or data")
            self.iv = __iv
            self.cipheredData = __cipheredData
            self.decipheredData = self.decrypt()
            if self.decipheredData is not None:
                self.decipheredData_int = ByteArr_ToIntArr(self.decipheredData)
            else:
                self.decipheredData_int = None

    def decrypt(self):
        """
        decrypts data if possible
        returns deciphered Data as Byte-Array
        """
        decipherVal = AES.new(self.key, AES.MODE_CBC, iv=self.iv)
        try:
            unpadded = unpad(decipherVal.decrypt(
                self.cipheredData), AES.block_size)
            self.decipherble = True
        except:
            unpadded = None
            self.decipherble = False
            print("ERR: could not be deciphered")
        return unpadded
