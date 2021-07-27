# How to download ai2thor documentation

## Why

Ai2Thor gets updated really quickly and their documentation
website does not have a way to view docs for previous versions.
Therefore it is a good idea to download the docs directly.

## How

1. Thanks to [this amazing answer](https://superuser.com/a/14436/439427),
   you can download a website by `wget`. Just run

   ```
   wget -r --no-parent https://ai2thor.allenai.org/ithor/documentation/
   ```

   This very quickly downloads the whole website (not sure how it does it)

   Expected output:
   ```
   ...
   --2021-07-27 09:00:16--  https://ai2thor.allenai.org/ithor/documentation/concepts
   Connecting to ai2thor.allenai.org (ai2thor.allenai.org)|185.199.110.153|:443... connected.
   HTTP request sent, awaiting response... 301 Moved Permanently
   Location: https://ai2thor.allenai.org/ithor/documentation/concepts/ [following]
   --2021-07-27 09:00:16--  https://ai2thor.allenai.org/ithor/documentation/concepts/
   Reusing existing connection to ai2thor.allenai.org:443.
   HTTP request sent, awaiting response... 200 OK
   Length: 474552 (463K) [text/html]
   Saving to: ‘ai2thor.allenai.org/ithor/documentation/concepts’

   ai2thor.allenai.org/ith 100%[=============================>] 463.43K  --.-KB/s    in 0.06s
   ```

2. Then, you want to edit the webpage file names, so that they have an extension of `.html`.
   The downloaded files don't have extensions.
   Doing so with emacs dired is very quick.

3. Then, set the `ITHOR_DOCS` variable in `server.py` to the directory that contains
   the `ithor` folder. That is where the documentation will be served.

4. Run `server.py`:
   ```
   $ python server.py
   Serving HTTP on 0.0.0.0 port 8000 (http://0.0.0.0:8000/) ...
   127.0.0.1 - - [27/Jul/2021 09:03:56] "GET / HTTP/1.1" 200 -
   127.0.0.1 - - [27/Jul/2021 09:03:58] "GET /ithor/ HTTP/1.1" 200 -
   ...
   ```
