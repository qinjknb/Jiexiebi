import urllib.request, os

url = 'https://github.com/openocd-org/openocd/releases/download/v0.12.0/openocd-v0.12.0-i686-w64-mingw32.tar.gz'
tgz = r'D:\GCWJ\tools\openocd.tar.gz'

print('downloading', url)
urllib.request.urlretrieve(url, tgz)
print('downloaded', os.path.getsize(tgz), 'bytes')
