import curses

def main(win):
    win.nodelay(True)
    win.keypad(True)
    key=""
    win.clear()
    win.addstr("Detected key:")
    last_key = None
    counter = 0
    done = False
    while done == False:
        try:
            key = win.getch()
            if key == -1:
                # No input
                continue
            win.clear()
            win.addstr("Detected key:")
            win.addstr(str(key))
            if key == last_key:
                counter += 1
            else:
                counter = 0
            last_key = key
            win.addstr(" ({})".format(counter))
            if key == 27: # push ESC key
                done = True
                break
            else:
                win.refresh()
        except Exception as e:
            # No input
            pass

curses.wrapper(main)
