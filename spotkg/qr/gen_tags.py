import io
import os
import zipfile
from typing import Dict

import qrcode
from PIL import Image, ImageDraw

from ..globals import ACTIONS

_LOGO = Image.open("./spotkg/logo.png")


def _add_info_to_qr(img, info):
    global _LOGO
    new_img = Image.new('RGBA', (img.size[0] + 100, img.size[1] + 100), (255, 255, 255))
    draw = ImageDraw.Draw(new_img)

    # draw the qr code
    new_img.paste(img, (50, 50))

    # draw logo above the qr code
    # header dimensions
    header_width = new_img.size[0]
    header_height = (new_img.size[1] - img.size[1]) / 2
    # scale logo to fit in header
    logo_width = header_width * 0.5
    logo_height = _LOGO.size[1] * (logo_width / _LOGO.size[0])
    logo = _LOGO.resize((int(logo_width), int(logo_height)))

    # paste logo in header
    new_img.paste(logo, (int((header_width - logo_width) / 2), int(header_height - logo_height / 2)), logo)

    # draw the info below the qr code centered
    info_width = new_img.size[0]
    info_height = (new_img.size[1] - img.size[1]) / 2
    draw.text((int((info_width - draw.textsize(info)[0]) / 2), int(info_height + img.size[1] + 10)), info, (0, 0, 0))
    return new_img


def gen_tags(path: str, tag_map: Dict[int, str] = None, make_zip: bool = False, add_info: bool = True) -> None:
    """Generate QR tags for the given actions."""
    if tag_map is None:
        tag_map = {i: c for i, c in enumerate(ACTIONS)}
    os.makedirs(path, exist_ok=True)
    if not make_zip:
        zip_file = None
        for cmd in ACTIONS:
            os.makedirs(f"{path}/{cmd}", exist_ok=True)
    else:
        zip_file = zipfile.ZipFile(f"{path}/tags.zip", "w")
    for i, cmd in tag_map.items():
        if cmd not in ACTIONS:
            raise ValueError(f"Invalid command {cmd}.")
        qr = qrcode.QRCode()
        qr.add_data(f"{i}:{cmd}")
        img: Image = qr.make_image()
        if add_info:
            img = _add_info_to_qr(img, f"{i} | {cmd}")
        if make_zip:
            output = io.BytesIO()
            img.save(output, format='png')
            zip_file.writestr(f"{cmd}/{i}_{cmd}.png", output.getvalue())
        else:
            img.save(f"{path}/{cmd}/{i}_{cmd}.png")
        print(f"{i}:{cmd}")
    if make_zip:
        print(f"Saved tags to {path}/tags.zip.")
        zip_file.close()


if __name__ == "__main__":
    tags = {i: ACTIONS[i % len(ACTIONS)] for i in range(3 * len(ACTIONS))}
    gen_tags("../../data/tags", tags, True)
    print("Done.")
