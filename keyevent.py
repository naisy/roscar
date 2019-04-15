import curses

def main(win):
    win.nodelay(True)
    key=""
    win.clear()
    win.addstr("Detected key:")
    last_key = None
    counter = 0
    while 1:
        try:
            key = win.getkey()
            win.clear()
            win.addstr("Detected key:")
            win.addstr(str(key))
            if key == last_key:
                counter += 1
            else:
                counter = 0
            last_key = key
            win.addstr(" ({})".format(counter))
            if key == os.linesep:
                break
        except Exception as e:
            # No input
            pass

curses.wrapper(main)
