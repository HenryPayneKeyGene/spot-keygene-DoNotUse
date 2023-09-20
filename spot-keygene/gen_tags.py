import io
import os
import zipfile

import qrcode
from qrcode.image.pil import PilImage

COMMANDS = ["scan", "upload"]


def gen_tags(path, tag_map=None, make_zip=False):
    if tag_map is None:
        tag_map = {i: c for i, c in enumerate(COMMANDS)}
    os.makedirs(path, exist_ok=True)
    if not make_zip:
        zip_file = None
        for cmd in COMMANDS:
            os.makedirs(f"{path}/{cmd}", exist_ok=True)
    else:
        zip_file = zipfile.ZipFile(f"{path}/tags.zip", "w")
    for i, cmd in tag_map.items():
        if cmd not in COMMANDS:
            raise ValueError(f"Invalid command {cmd}.")
        qr = qrcode.QRCode()
        qr.add_data(f"{i}:{cmd}")
        img: PilImage = qr.make_image()
        if make_zip:
            output = io.BytesIO()
            img.save(output, format='png')
            print(len(output.getvalue()))
            zip_file.writestr(f"{cmd}/{i}_{cmd}.png", output.getvalue())
        else:
            img.save(f"{path}/{cmd}/{i}_{cmd}.png")
    if make_zip:
        zip_file.close()


if __name__ == "__main__":
    tags = {i: COMMANDS[i % len(COMMANDS)] for i in range(6)}
    gen_tags("../tags", tags, True)
    print("Done.")
