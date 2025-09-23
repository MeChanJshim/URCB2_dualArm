import numpy as np
import matplotlib.pyplot as plt



data = np.loadtxt("cmd_continue6D.txt")


num_rows, num_cols = data.shape

print(f"Loaded data was {num_rows} rows, {num_cols} cols.")

# Postion Plot
plt.figure(figsize=(10,6))
for i in range(3):
    plt.plot(data[:,i], label=f"column {i+1}")

plt.title("Positions of cmd_continue6D.txt")
plt.xlabel("Row Index")
plt.ylabel("Value")
plt.legend()
plt.grid(True)
plt.tight_layout()

# Orientation Plot
plt.figure(figsize=(10,6))
for i in range(3):
    plt.plot(data[:,i+3], label=f"column {i+1}")

plt.title("Orientations of cmd_continue6D.txt")
plt.xlabel("Row Index")
plt.ylabel("Value")
plt.legend()
plt.grid(True)
plt.tight_layout()

# Force Plot
plt.figure(figsize=(10,6))
for i in range(3):
    plt.plot(data[:,i+6], label=f"column {i+1}")

plt.title("Force of cmd_continue6D.txt")
plt.xlabel("Row Index")
plt.ylabel("Value")
plt.legend()
plt.grid(True)
plt.tight_layout()



plt.show()