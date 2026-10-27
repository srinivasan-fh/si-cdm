#!/usr/bin/env python3
"""Lists every word the interface looks up with tr("..."), for a translator.

    tools/i18n_extract.py                      the words, one a line
    tools/i18n_extract.py --json de.json       a language file to fill in: the words of an
                                               existing de.json are kept, new ones added empty
    tools/i18n_extract.py --missing de         the words the built-in translation lacks
                                               (src/core/lang/*.cpp); exits 1 when there are any

Install a finished file as <config directory>/lang/<code>.json (see docs/translation.md).
"""
import json, os, re, sys

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
CALL = re.compile(r'\b(?:tr|SICDM_TR_NOOP)\(((?:\s*"(?:[^"\\]|\\.)*")+)\)')
LIT = re.compile(r'"((?:[^"\\]|\\.)*)"')
BUILTIN = {'de': 'German.cpp'}


def unescape(c):
    """A C++ string literal's text (the escapes the sources use; \\xHH bytes are UTF-8)."""
    out, i = bytearray(), 0
    while i < len(c):
        if c[i] == '\\' and i + 1 < len(c):
            e = c[i + 1]
            if e == 'x':
                j = i + 2
                while j < len(c) and j < i + 4 and c[j] in '0123456789abcdefABCDEF':
                    j += 1
                out.append(int(c[i + 2:j], 16))
                i = j
                continue
            out += {'n': '\n', 't': '\t', '"': '"', '\\': '\\'}.get(e, e).encode('utf-8')
            i += 2
        else:
            out += c[i].encode('utf-8')
            i += 1
    return out.decode('utf-8', 'replace')


def builtin(code):
    path = os.path.join(ROOT, 'src', 'core', 'lang', BUILTIN[code])
    with open(path, encoding='utf-8') as fh:
        text = fh.read()
    pairs = re.findall(r'\{"((?:[^"\\]|\\.)*)", "((?:[^"\\]|\\.)*)"\}', text)
    return {unescape(a): unescape(b) for a, b in pairs}


def words():
    found = set()
    for top in ('src',):
        for dirpath, _, files in os.walk(os.path.join(ROOT, top)):
            for f in files:
                if not f.endswith(('.cpp', '.hpp')):
                    continue
                with open(os.path.join(dirpath, f), encoding='utf-8') as fh:
                    for m in CALL.finditer(fh.read()):
                        w = ''.join(unescape(x) for x in LIT.findall(m.group(1))).split('##')[0].strip(' ')
                        if w:
                            found.add(w)
    return sorted(found)


def main():
    ws = words()
    if len(sys.argv) >= 3 and sys.argv[1] == '--missing':
        have = builtin(sys.argv[2])
        # A word that reads the same in the language needn't be in the table: only words with
        # letters count, and those must be there.
        missing = [w for w in ws if w not in have and re.search(r'[A-Za-z]{2,}', re.sub(r'%[-+ #0-9.]*[a-zA-Z]', '', w))]
        same = {'Claude Code:', 'Dyntopo', 'Linear', 'MCP  :%d', 'Name', 'OK', 'Parallel', 'Plugins', 'Pose', 'Radius', 'Retopo',
                'Start', 'UV', 'Updates', 'mm', 'Map', 'Magnet', 'Detail', 'Dear ImGui %s', 'OpenGL   : %s', 'Renderer : %s', 'SI CDM  %s'}
        missing = [w for w in missing if w not in same]
        for w in missing:
            print(w)
        print('%d words, %d without a %s translation' % (len(ws), len(missing), sys.argv[2]), file=sys.stderr)
        sys.exit(1 if missing else 0)
    if len(sys.argv) >= 3 and sys.argv[1] == '--json':
        path = sys.argv[2]
        doc = {'name': '', 'strings': {}}
        if os.path.exists(path):
            with open(path, encoding='utf-8') as fh:
                doc = json.load(fh)
        strings = doc.setdefault('strings', {})
        for w in ws:
            strings.setdefault(w, '')
        with open(path, 'w', encoding='utf-8') as fh:
            json.dump(doc, fh, ensure_ascii=False, indent=1, sort_keys=True)
        missing = sum(1 for w in ws if not strings.get(w))
        print('%s: %d words, %d still to translate' % (path, len(ws), missing))
    else:
        for w in ws:
            print(w)


if __name__ == '__main__':
    main()
