"""Fetch the X-Plane plugin SDK (XPSDK420) into the given directory.

The SDK is Laminar Research's, under a BSD-style licence; it is fetched at
build time rather than vendored into this repo.
"""
import io
import sys
import urllib.request
import zipfile

URL = "https://developer.x-plane.com/wp-content/plugins/code-sample-generation/sdk_zip_files/XPSDK420.zip"

dest = sys.argv[1] if len(sys.argv) > 1 else "sdk"
req = urllib.request.Request(URL, headers={"User-Agent": "xplane-bench-build"})
with urllib.request.urlopen(req, timeout=60) as r:
    data = r.read()
zipfile.ZipFile(io.BytesIO(data)).extractall(dest)
print(f"XPSDK420 -> {dest}")
