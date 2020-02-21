#!/usr/bin/env python

import argparse
import multiprocessing

import jsk_data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--verbose', dest='quiet', action='store_false')
    args = parser.parse_args()
    quiet = args.quiet

    def download_data(**kwargs):
        kwargs['pkg_name'] = 'cape_ros'
        kwargs['quiet'] = quiet
        p = multiprocessing.Process(
            target=jsk_data.download_data,
            kwargs=kwargs)
        p.start()

    download_data(
        path='sample/data/2020-02-22-hsr-move-home-environment.bag',
        url='https://drive.google.com/uc?id=1pNbrVoccbEbQzF02ZesCYChv-_39WHHM',
        md5='1f09f3d8e0ffc74428b6340c61eb93fd',
        extract=False,
    )


if __name__ == '__main__':
    main()
