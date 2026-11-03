import json, urllib.request, zipfile, os, sys

dest = r'D:\GCWJ\tools\pylibs'
os.makedirs(dest, exist_ok=True)

def get_wheel(pkg, suffix):
    with urllib.request.urlopen(f'https://pypi.org/pypi/{pkg}/json', timeout=30) as r:
        data = json.load(r)
    for u in data['urls']:
        fn = u['filename']
        if fn.endswith('.whl') and suffix in fn:
            return u['url'], fn
    return None, None

for pkg, suffix in [('pystlink', 'py3-none-any'), ('pyusb', 'py3-none-any'), ('libusb', 'py3-none-any')]:
    url, fn = get_wheel(pkg, suffix)
    print(pkg, '->', fn)
    if not url:
        print('  NOT FOUND'); continue
    wheel_path = os.path.join(dest, fn)
    urllib.request.urlretrieve(url, wheel_path)
    print('  downloaded', os.path.getsize(wheel_path), 'bytes')
    with zipfile.ZipFile(wheel_path) as z:
        z.extractall(dest)
    print('  extracted')

print('pylibs contents:', sorted(os.listdir(dest)))
