import json, urllib.request, zipfile, os

dest = r'D:\GCWJ\tools\pylibs'
os.makedirs(dest, exist_ok=True)

with urllib.request.urlopen('https://pypi.org/pypi/pypdf/json', timeout=30) as r:
    data = json.load(r)

url = None
for u in data['urls']:
    if u['filename'].endswith('-py3-none-any.whl'):
        url = u['url']
        break
print('wheel url:', url)

wheel_path = os.path.join(dest, os.path.basename(url))
urllib.request.urlretrieve(url, wheel_path)
print('downloaded', os.path.getsize(wheel_path), 'bytes')

with zipfile.ZipFile(wheel_path) as z:
    z.extractall(dest)
print('extracted to', dest)
print(sorted(os.listdir(dest)))
