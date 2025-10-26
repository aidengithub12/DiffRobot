path = 'C:\\Users\\HP\\OneDrive\\Documents\\Github\\DiffRobot\\Data.csv'
def getData():
    with open(path,"r") as file:
        content = file.read()
        print(file.readable())
        print(content)
        file.close()
    return content
def writeData(writeData: str):
    with open(path, "w") as file:
        file.write(writeData)