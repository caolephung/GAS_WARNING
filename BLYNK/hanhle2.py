import seaborn as sns
import pandas as pd
import matplotlib.pyplot as plt

'''
Load the data
'''
# Load built-in 'tips' dataset
tips = sns.load_dataset('tips')
tips.head()

'''
Create a graph of the relationship between the total bill and the tip
'''
sns.scatterplot(data=tips, x='total_bill', y='tip')
plt.title("Tip vs Total Bill")
plt.xlabel("Total Bill ($)")
plt.ylabel("Tip ($)")
plt.grid(True)
plt.show()


'''
Redo 2 with the tip as a percentage of the total_bill
'''
# Add tip percentage column
tips['tip_pct'] = tips['tip'] / tips['total_bill'] * 100

# Plot
sns.scatterplot(data=tips, x='total_bill', y='tip_pct')
plt.title("Tip % vs Total Bill")
plt.xlabel("Total Bill ($)")
plt.ylabel("Tip (%)")
plt.grid(True)
plt.show()

'''
How does the tip (as % of total bill) relate to the size of the group?
'''
sns.boxplot(data=tips, x='size', y='tip_pct')
plt.title("Tip % vs Group Size")
plt.xlabel("Group Size")
plt.ylabel("Tip (%)")
plt.grid(True)
plt.show()

'''
Aggregate the data and compute the median tip % based on time (Lunch or Dinner)
'''
tips['tip_percent'] = 100 * tips['tip'] / tips['total_bill'] 
median_tip_by_time = tips.groupby('time', observed=False)['tip_percent'].median().reset_index()
print(median_tip_by_time)

'''
Run a regression that estimates the tip based on the total_bill. What does the intercept reveal?
'''
import statsmodels.api as sm

X = tips['total_bill']
y = tips['tip']

# Constant cho intercept
X = sm.add_constant(X)

# Fit model
model = sm.OLS(y, X).fit()
print(model.summary())
