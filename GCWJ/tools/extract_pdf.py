import sys
sys.path.insert(0, r'D:\GCWJ\tools\pylibs')
from pypdf import PdfReader

reader = PdfReader(r'D:\GCWJ\德晟串口智能舵机协议文件 v4.01.pdf')
out = []
for i, page in enumerate(reader.pages, 1):
    text = page.extract_text() or ''
    out.append(f'===== PAGE {i} =====\n{text}')

with open(r'D:\GCWJ\tools\protocol-text.txt', 'w', encoding='utf-8') as f:
    f.write('\n'.join(out))
print('pages:', len(reader.pages))
print('total chars:', sum(len(o) for o in out))
