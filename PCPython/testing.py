# first source file
fp1 = open("7.wav", "rb")
# second source file
fp2 = open("8.wav", "rb")
# result source file
fp3 = open("9.wav", "wb")

# file 1.wav is smaller!!!

byte1 = fp1.read(2)
byte2 = fp2.read(2)
print(byte1.__len__())

i = 0
while byte2:
    # mix audio data, starting point
    if i >= 280 and byte1:
        print(byte1)
        v1 = int.from_bytes(byte1, byteorder='little', signed=True)
        v2 = int.from_bytes(byte2, byteorder='little', signed=True)
        sum = v1 + v2
        if sum < -32768:
            sum = -32768
        if sum > 32767:
            sum = 32767
        sum = sum.to_bytes(2, byteorder='little', signed=True)
        # fp3.write(sum)
    else:
        fp3.write(byte2)

    if i % 1000000 == 0:
        print(i / 1000000)

    byte1 = fp1.read(2)
    byte2 = fp2.read(2)
    i += 1

print("end")
