import numpy as np
import numpy_financial as npf
import matplotlib.pyplot as plt
from scipy.optimize import brentq

# Khởi tạo dòng tiền
initial_investment = -100000
cf1 = [initial_investment, 5000, 4000, 2000, 3000, 1000]
cf2 = [initial_investment, 2000, 3000, 2000, 4000, 5000]

'''
If the discount rate is 10%, calculate the Net Present Value (NPV) and the Internal Rate of Return (IRR)
for each project. If the company can only choose one project, which one should it select?
'''
# Hàm tính NPV
def calculate_npv(rate, cashflows):
    return npf.npv(rate, cashflows)

# Hàm tính IRR
def calculate_irr(cashflows):
    return npf.irr(cashflows)

# Tính NPV và IRR ở mức chiết khấu 10%
rate = 0.10
npv1 = calculate_npv(rate, cf1)
npv2 = calculate_npv(rate, cf2)
irr1 = calculate_irr(cf1)
irr2 = calculate_irr(cf2)

# In kết quả
print(f"Project 1: NPV = ${npv1:.2f}, IRR = {irr1:.4f} ({irr1*100:.2f}%)")
print(f"Project 2: NPV = ${npv2:.2f}, IRR = {irr2:.4f} ({irr2*100:.2f}%)")
print("=> Selected project:", "Project 1" if npv1 > npv2 else "Project 2")

'''
Visualize the two cash flows on the same coordinate system.
'''
# Vẽ dòng tiền
years = np.arange(0, len(cf1))
plt.figure(figsize=(10, 5))
plt.plot(years, cf1, marker='o', label="Project 1")
plt.plot(years, cf2, marker='s', label="Project 2")
plt.axhline(0, color='gray', linestyle='--')
plt.title("Cash Flows for Projects 1 and 2")
plt.xlabel("Year")
plt.ylabel("Cash Flow")
plt.legend()
plt.grid(True)
plt.show()

'''
Write a function to determine the discount rate at which both projects have equal NPV.
'''
# Tìm discount rate mà NPV hai dự án bằng nhau
equal_rate = find_equal_npv_rate(cf1, cf2)
print(f"Equal NPV discount rate = {equal_rate:.4f} or {equal_rate*100:.2f}%")

'''
Plot the NPV profiles of the two projects using Python.
'''
# Vẽ NPV profile
rates = np.linspace(0, 0.3, 100)
npv_profile1 = [calculate_npv(r, cf1) for r in rates]
npv_profile2 = [calculate_npv(r, cf2) for r in rates]

plt.figure(figsize=(10, 5))
plt.plot(rates * 100, npv_profile1, label="Project 1")
plt.plot(rates * 100, npv_profile2, label="Project 2")
plt.axhline(0, color='gray', linestyle='--')
plt.axvline(equal_rate * 100, color='purple', linestyle=':', label=f'Equal NPV rate\n({equal_rate*100:.2f}%)')
plt.title("NPV Profiles of Projects")
plt.xlabel("Discount Rate (%)")
plt.ylabel("NPV ($)")
plt.legend()
plt.grid(True)
plt.show()
