import matplotlib.pyplot as plt

data_str = """
t:10,RPMA:0.00,RPMB:0.00,
t:20,RPMA:33.00,RPMB:6.00,
t:30,RPMA:30.43,RPMB:26.57,
t:40,RPMA:46.29,RPMB:49.71,
t:50,RPMA:65.14,RPMB:80.57,
t:60,RPMA:78.00,RPMB:96.43,
t:70,RPMA:90.00,RPMB:114.86,
t:80,RPMA:103.29,RPMB:134.57,
t:90,RPMA:112.71,RPMB:136.29,
t:100,RPMA:120.86,RPMB:121.29,
t:110,RPMA:132.00,RPMB:85.71,
t:120,RPMA:137.14,RPMB:96.43,
t:130,RPMA:147.86,RPMB:117.86,
t:140,RPMA:151.29,RPMB:140.57,
t:150,RPMA:159.43,RPMB:124.71,
t:160,RPMA:163.71,RPMB:121.71,
t:170,RPMA:164.57,RPMB:115.71,
t:180,RPMA:170.57,RPMB:120.00,
t:190,RPMA:175.71,RPMB:116.14,
t:200,RPMA:178.29,RPMB:117.00,
t:210,RPMA:176.14,RPMB:117.86,
t:220,RPMA:175.71,RPMB:140.57,
t:230,RPMA:183.00,RPMB:121.71,
t:240,RPMA:179.57,RPMB:118.71,
t:250,RPMA:185.57,RPMB:111.86,
t:260,RPMA:192.43,RPMB:107.57,
t:270,RPMA:189.43,RPMB:109.29,
t:280,RPMA:194.14,RPMB:116.14,
t:290,RPMA:197.14,RPMB:129.86,
t:300,RPMA:197.57,RPMB:119.14,
t:310,RPMA:197.57,RPMB:114.00,
t:320,RPMA:198.00,RPMB:121.71,
t:330,RPMA:202.29,RPMB:118.71,
t:340,RPMA:198.43,RPMB:127.29,
t:350,RPMA:199.29,RPMB:118.29,
t:360,RPMA:203.57,RPMB:105.86,
t:370,RPMA:201.86,RPMB:112.29,
t:380,RPMA:198.86,RPMB:122.14,
t:390,RPMA:199.29,RPMB:136.71,
t:400,RPMA:195.86,RPMB:162.43,
t:410,RPMA:189.00,RPMB:159.43,
t:420,RPMA:192.00,RPMB:146.14,
t:430,RPMA:191.57,RPMB:120.86,
t:440,RPMA:185.57,RPMB:127.29,
t:450,RPMA:186.43,RPMB:114.43,
t:460,RPMA:188.14,RPMB:114.86,
t:470,RPMA:177.86,RPMB:137.14,
t:480,RPMA:176.57,RPMB:152.57,
t:490,RPMA:174.43,RPMB:159.86,
t:500,RPMA:162.00,RPMB:154.71,
t:510,RPMA:158.14,RPMB:138.00,
t:520,RPMA:154.29,RPMB:116.14,
t:530,RPMA:150.43,RPMB:110.14,
t:540,RPMA:144.86,RPMB:90.86,
t:550,RPMA:135.86,RPMB:89.14,
t:560,RPMA:129.86,RPMB:117.00,
t:570,RPMA:126.86,RPMB:146.57,
t:580,RPMA:121.29,RPMB:136.71,
t:590,RPMA:116.14,RPMB:105.43,
t:600,RPMA:110.57,RPMB:92.14,
"""

# Splitting the data into lines
lines = data_str.strip().split('\n')

# Creating a list of tuples
data = [
    (
        int(line.split(',')[0].split(':')[1]),  # Extracting timestamp
        float(line.split(',')[1].split(':')[1]),  # Extracting RPMA value
        float(line.split(',')[2].split(':')[1])   # Extracting RPMB value
    )
    for line in lines
]

# Provided data


# Extracting data for RPMA and RPMB
timestamps, rpma_values, rpmb_values = zip(*data)

# Dividing timestamps by 100 to convert them to seconds
timestamps = [t / 100 for t in timestamps]

# Plotting the data
plt.plot(timestamps, rpma_values, label='RPMA', marker='o', linestyle='dashed')
plt.plot(timestamps, rpmb_values, label='RPMB', marker='o', linestyle='dashed')

# Adding labels and title
plt.xlabel('Time (s)')
plt.ylabel('RPM')
plt.title('RPM vs Time')
plt.legend()
plt.grid(True)

# Show the plot
plt.show()