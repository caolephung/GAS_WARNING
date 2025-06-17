import pandas as pd

'''
Load the data onto Visual Studio Code
'''
# Load CRSP-style dataset
crsp_df = pd.read_csv('datanew.csv')

# Load Fama-French 5 factors dataset
ff_df = pd.read_csv('ff_5factors_website.csv')

# Check data
print(crsp_df.head())
print(ff_df.head())

'''
Convert the date into pandas datetime format
'''
crsp.rename(columns={
    'DATE': 'date',
    'RET': 'ret',
    'PERMNO': 'permno',
    'SIC': 'sic'
}, inplace=True)

crsp_df['date'] = pd.to_datetime(crsp_df['date'])

# Hợp nhất dữ liệu theo cột 'date'
merged = pd.merge(crsp, ff, on='date', how='inner')

# Kiểm tra dữ liệu sau khi gộp
print(merged.head())

'''
Create equally weighted portfolios by SIC code
'''
# Tạo cột 'industry' từ mã SIC
def classify_industry(sic):
    try:
        sic = int(sic)
        if 100 <= sic <= 999: return 'Mining'
        elif 1000 <= sic <= 1499: return 'Construction'
        elif 2000 <= sic <= 3999: return 'Manufacturing'
        elif 4000 <= sic <= 4999: return 'Transportation'
        elif 5000 <= sic <= 5199: return 'Wholesale'
        elif 5200 <= sic <= 5999: return 'Retail'
        elif 6000 <= sic <= 6799: return 'Finance'
        elif 7000 <= sic <= 8999: return 'Services'
        else: return 'Non-classifiable'
    except:
        return 'Non-classifiable'

merged_df['industry'] = merged_df['sic'].apply(classify_industry)

# Tính return trung bình theo industry và tháng 
industry_returns = merged_df.groupby(['month', 'industry'])['ret'].mean().reset_index()
