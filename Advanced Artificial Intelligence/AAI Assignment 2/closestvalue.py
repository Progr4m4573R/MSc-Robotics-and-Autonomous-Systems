myList = [1.5,2.5,3.5,4.5,5.5,6.5,7.5]
myNumber = 9
closest = min(myList, key=lambda x:abs(x-myNumber))
print(closest)