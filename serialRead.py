# Copyright (C) 2022 Wren Fannin and Arav Raja luwr5102@gmail.com 
# Everyone is permitted to copy and distribute copies of this file under GNU-GPL3

import serial as ser
import time
import bitdotio
import pandas as pd

b = bitdotio.bitdotio('v2_3sAPd_rrU3xSavWUNjwEdxj8LbVMH')

df_test = pd.DataFrame(
    data=[[0, 1, 2], [3, 4, 5]],
    columns=['a', 'b', 'c'])
df_test.to_csv('test.csv', index=False)
create_table_sql = """
    CREATE TABLE test (
      a integer,
      b integer,
      c integer
    )
    """
with b.get_connection("AirQuality") as conn:
    cursor = conn.cursor()
    cursor.execute(create_table_sql)


copy_table_sql = """
    COPY test FROM stdin WITH CSV HEADER DELIMITER as ',';
    """

with open('test.csv', 'r') as f:
    with b.get_connection("<YOUR_DATABASE_NAME>") as conn:
        cursor = conn.cursor()
        cursor.copy_expert(sql=copy_table_sql, file=f)

      
ser = serial.Serial('/dev/ttyACM0',115200, timeout=10) # IMPORTANT, CHANGE ACM0 to USB0 OR OTHER AS APPROPIRATE
ser.open()
data = ser.read()


while True:
    new_file = open('info_file.txt', 'a')
    new_file.write(data)
# Unfinished, more functionality is being added to write this data into a database
