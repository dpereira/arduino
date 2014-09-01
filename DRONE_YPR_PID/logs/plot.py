import csv
import matplotlib.pyplot as plt

reader = csv.reader(open("plot.csv"))
data = [(r[1], r[4]) for r in  reader]
print data

plt.plot(data)
plt.show()


