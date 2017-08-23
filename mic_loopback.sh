#arecord -D plughw:1,0 -f CD --period-time 10 --buffer-time 4000 | aplay -D plughw:0,0 &
