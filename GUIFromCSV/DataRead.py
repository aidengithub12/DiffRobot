path = 'C:\\Users\\HP\\OneDrive\\Documents\\Github\\DiffRobot\\Data.csv'
def getData():
    with open(path,"r") as file:
        content = file.read()
        file.close()
    return content
def writeData(writeData: str):
    with open(path, "w") as file:
        file.write(writeData)
def parseData(data: str):
    seperated_data = []
    temp_str = ""
    for i in range(0,len(data)):
        if data[i] != ',':
            temp_str = temp_str + data[i]
        else:
            seperated_data.append(temp_str)
            temp_str = ""
    return seperated_data
