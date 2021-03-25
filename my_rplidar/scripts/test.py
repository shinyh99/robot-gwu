print("hello")

ls = [10, 20]
ls = ls + [30]
ls.append([40])

ls = range(1, 4)

print(ls)


def divideInto(aList):
    half = len(aList) // 2
    leftHalf = aList[half:]
    rightHalf = aList[:half]
    if len(aList) == 1:
        return [aList[0]]
    else:
        return divideInto(leftHalf) + divideInto(rightHalf)


ls2 = divideInto(ls)

print(ls2)