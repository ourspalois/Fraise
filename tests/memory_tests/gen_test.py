import numpy as np 

def test_control_regs(filename) : 
    
    control_base_address = 128
    control_lenght = 8
    values = np.random.randint(0, 2 ** 8 - 1, size=control_lenght, dtype=np.uint8)

    with open(filename, 'a') as file:
        for address in range(control_base_address, control_base_address + control_lenght):
            file.write(f"write | {address :02X} | {values[address - control_base_address] :02X}\n")

        for address in range(control_base_address, control_base_address + control_lenght):
            file.write(f"read | {address :02X} | {values[address - control_base_address] :02X}\n")

def test_write_read(fielname):

    Array = np.random.randint(0, 2 ** 8 - 1, size=(16, 8), dtype=np.uint8)

    with open(filename, 'a') as file:
        file.write("write | 80 | 00\n")
        for array in range(Array.shape[0]):
            for line in range(Array.shape[1]):
                adress = line + 8 * array
                file.write(f"write | {adress :02X} | {Array[array, line] :02X}\n")

        file.write("write | 80 | 01\n")
        for array in range(Array.shape[0]):
            for line in range(Array.shape[1]):
                adress = line + 8 * array
                file.write(f"write | {adress :02X} | {Array[array, line] :02X}\n")
        
        for array in range(Array.shape[0]):
            for line in range(Array.shape[1]):
                adress = line + 8 * array
                file.write(f"read | {adress :02X} | {Array[array, line] :02X}\n")

def test_inference(filename, num_test, overflow_test=False):
    if overflow_test : 
        Array = np.random.randint(0, 2 ** 8 - 1, size=(16, 8), dtype=np.uint8)
    else :
        Array = np.random.randint(0, 2 ** 6 - 1, size=(16, 8), dtype=np.uint8)

    with open(filename, 'a') as file:
        # write the arrays 
        file.write("write | 80 | 00\n")
        for array in range(Array.shape[0]):
            for line in range(Array.shape[1]):
                adress = line + 8 * array
                file.write(f"write | {adress :02X} | {Array[array, line] :02X}\n")

        file.write("write | 80 | 01\n")
        for array in range(Array.shape[0]):
            for line in range(Array.shape[1]):
                adress = line + 8 * array
                file.write(f"write | {adress :02X} | {Array[array, line] :02X}\n")

        # run inferences 

        def inference_log( obs1, obs2, obs3, obs4, array) : 
            array = array.astype(np.uint16)
            file.write(f"write | 81 | {obs1:02X}\n")
            file.write(f"write | 82 | {obs2:02X}\n")
            file.write(f"write | 83 | {obs3:02X}\n")
            file.write(f"write | 84 | {obs4:02X}\n")
            expected_data0 = array[0][obs1] + array[1][obs2] + array[2][obs3] + array[3][obs4]
            expected_data1 = array[4][obs1] + array[5][obs2] + array[6][obs3] + array[7][obs4]
            expected_data2 = array[8][obs1] + array[9][obs2] + array[10][obs3] + array[11][obs4]
            expected_data3 = array[12][obs1] + array[13][obs2] + array[14][obs3] + array[15][obs4]
            expected_array = np.array([expected_data0, expected_data1, expected_data2, expected_data3], dtype=np.uint32)
            
            expected_clip = np.clip(expected_array, 0, 255)
            expected_data = expected_clip[0] | expected_clip[1] << 8 | expected_clip[2] << 16 | expected_clip[3] <<24
            file.write(f"read | 88 | {expected_data:08X}\n")

        test_points = np.random.randint(0, 8, (num_test, 4))
        for i in range(num_test):
            obs1 = test_points[i][0]
            obs2 = test_points[i][1]
            obs3 = test_points[i][2]
            obs4 = test_points[i][3]
            inference_log(obs1, obs2, obs3, obs4, Array)

            
def insert_reset(filename):
    with open(filename, 'a') as file:
        file.write("reset\n")


if __name__ == "__main__":
    filename = "test_vectors.txt"
    import os 
    if os.path.exists(filename):
        open(filename, 'w').close()

    np.random.seed(0)
    test_control_regs(filename)
    insert_reset(filename)  
    test_write_read(filename)
    test_inference(filename, 1000)

