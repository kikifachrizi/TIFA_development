#!/usr/bin/env python3
import rospy
import os
import requests

API_URL = "https://be.tifa-app.my.id/api/images"
IMAGE_FOLDER = "/home/tifa/catkin_ws/src/tifa_nav/maps"

def upload_image(image_path):
    image_name = os.path.basename(image_path)
    # Deteksi MIME type
    if image_name.lower().endswith('.png'):
        mime_type = 'image/png'
    elif image_name.lower().endswith(('.jpg', '.jpeg')):
        mime_type = 'image/jpeg'
    else:
        mime_type = 'application/octet-stream'
    try:
        with open(image_path, 'rb') as f:
            files = {'image': (image_name, f, mime_type)}
            response = requests.post(API_URL, files=files)
        if response.status_code in [200, 201]:
            rospy.loginfo(f"[UPLOAD] Sukses upload {image_name} ke server! Response: {response.text}")
            return True
        else:
            rospy.logwarn(f"[UPLOAD] Gagal upload {image_name}, status: {response.status_code}, pesan: {response.text}")
            return False
    except Exception as e:
        rospy.logerr(f"[ERROR] Gagal upload {image_name}: {e}")
        return False

def main():
    rospy.init_node('upload_image_node')
    rospy.loginfo("Node upload_image_node aktif.")

    image_list = [
        os.path.join(IMAGE_FOLDER, f)
        for f in os.listdir(IMAGE_FOLDER)
        if f.lower().endswith(('.png', '.jpg', '.jpeg'))
    ]

    if not image_list:
        rospy.logwarn(f"Tidak ditemukan gambar .png/.jpg/.jpeg di folder: {IMAGE_FOLDER}")
    else:
        for img_path in image_list:
            rospy.loginfo(f"Proses upload {img_path} ...")
            upload_image(img_path)

    rospy.loginfo("Selesai upload semua gambar.")

if __name__ == '__main__':
    main()
