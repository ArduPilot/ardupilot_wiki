#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
Script to get last blog entries on Discourse (https://discuss.ardupilot.org/)

'''
import argparse
import urllib.request
import json
import sys
import os
import re
from bs4 import BeautifulSoup # to find youtube links

parser = argparse.ArgumentParser(description="python3 get_blog_posts.py [Number of posts to retrieve]")
parser.add_argument("--n_posts", dest='n_posts', default="8", help="Number of posts to retrieve")
parser.add_argument('--verbose', dest='verbose', action='store_false', default=True, help="show debugging output")
args = parser.parse_args()

BLOG_DISCOURSEURL = "https://discuss.ardupilot.org/c/blog/110.json"
NEWS_DISCOURSEURL = "https://discuss.ardupilot.org/latest.json"
files_names = {BLOG_DISCOURSEURL: './frontend/blog_posts.json', NEWS_DISCOURSEURL: './frontend/news_posts.json'}
error_count = 0


def clean_html(raw_html):
    cleanr = re.compile('<.*?>')
    clean_text = re.sub(cleanr, '', raw_html)
    return clean_text


def debug(str_to_print):
    """ Debug output if verbose is set. """
    if args.verbose:
        print("[get_blog_posts.py] " + str(str_to_print))


def error(str_to_print):
    """ Show and count the errors. """

    global error_count
    error_count += 1
    print("[get_discourse_posts.py][error]: " + str(str_to_print))


def get_posts(url):
    """ Download Discourse last posts page in JSON format. """

    debug('Requesting url ' + url + ' ... ')
    try:
        request = urllib.request.Request(url)
        request = urllib.request.urlopen(request).read()
        content = json.loads(request.decode('utf-8'))
    except Exception as e:
        error(e)
        sys.exit(1)
    finally:
        return content


def get_single_post_text(url):
    """ Download Discourse specific post in JSON format. """

    debug('Requesting post text ' + url + ' ... ')
    try:
        request = urllib.request.Request(url)
        request = urllib.request.urlopen(request).read()
        content = json.loads(request.decode('utf-8'))
    except Exception as e:
        error(e)
        sys.exit(1)
    finally:
        post_text = clean_html(str(content['post_stream']['posts'][0]['cooked']))
        # removing \n on the text
        item = post_text.split('\n')
        item = " ".join(item)
        # TO-DO: removing multiple white spaces due the \n removal
        #
        # returning twitter style string
        return str(item[0:140] + ' (...)')


def get_first_youtube_link(url):
    debug('Requesting post text ' + url + ' to look for youtube link... ')
    try:
        request = urllib.request.Request(url)
        request = urllib.request.urlopen(request).read()
    except Exception as e:
        error(e)
        sys.exit(1)
    finally:
        soup = BeautifulSoup(request.decode('utf-8'), "html.parser")
        for link in soup.find_all('a', href=True):
            if 'youtube' in str(link['href']):
                return str(link['href'])


def youtube_link_to_embed_link(url):
    return str(url).replace('https://www.youtube.com/watch?v=', 'https://www.youtube-nocookie.com/embed/')


def save_posts_to_json(url):
    """ Save last N posts from blog to the JSON file. """

    content = get_posts(url)
    data = []
    youtube_link = ''
    # TO-DO: run these url gets in parallel
    for i in range(1, int(args.n_posts) + 1):
        item = content['topic_list']['topics'][i]
        single_post_link = str('https://discuss.ardupilot.org/t/' + str(item['slug']) + '/' + str(item['id']))
        single_post_text = get_single_post_text(single_post_link + '.json')

        if str(item['image_url']) == 'None':
            has_image = False
            youtube_link = youtube_link_to_embed_link(get_first_youtube_link(single_post_link))
        else:
            has_image = True
            youtube_link = 'nops'

        data.append({'title': item['title'], 'image': item['image_url'], 'has_image': has_image,
                    'youtube_link': youtube_link, 'link': single_post_link, 'text': single_post_text.strip()})
    try:
        target_file = os.path.join(os.getcwd(), files_names[url])
        with open(target_file, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=4)
    except Exception as e:
        error(e)
        sys.exit(1)


debug('Starting...')
save_posts_to_json(BLOG_DISCOURSEURL)
save_posts_to_json(NEWS_DISCOURSEURL)
