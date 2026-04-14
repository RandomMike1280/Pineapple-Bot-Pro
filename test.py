txt = '''private val FIXED_COORDINATES_MM = listOf(
    Offset(580f, 722f),
    Offset(670f, 722f),
    Offset(760f, 722f),
    Offset(851f, 722f),
    Offset(941f, 722f),
    Offset(1032f, 722f),
    Offset(580f, 900f),
    Offset(670f, 900f),
    Offset(760f, 900f),
    Offset(851f, 900f),
    Offset(941f, 900f),
    Offset(1032f, 900f),
    Offset(658f, 1085f),
    Offset(762f, 1085f),
    Offset(865f, 1085f),
    Offset(969f, 1085f),
    Offset(1073f, 1085f),
    Offset(1176f, 1085f),
)'''

split = txt.split('\n')

for i in split:
    if i.startswith('    Offset('):
        i = i.replace('    Offset(', '')
        i = i.replace('f),', '')
        i = i.replace('f,', '')
        i = list(map(int, i.split()))
        print(f'Offset({i[0]-20}f, {i[1]+20}f),')