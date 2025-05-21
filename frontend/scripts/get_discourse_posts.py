#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script to get last blog entries on Discourse (https://discuss.ardupilot.org/)
"""
import argparse
import json
import re
import hashlib
import os
import platform
from pathlib import Path

import requests
from dataclasses import dataclass
from typing import List, Any, Tuple


class RequestExecutionError(Exception):
    pass


class WriteToFileError(Exception):
    pass


@dataclass
class Post:
    title: str
    image: str
    has_image: bool
    youtube_link: str
    link: str
    text: str


class BlogPostsFetcher:
    def __init__(self, blog_url: str, news_url: str):
        self.blog_url = blog_url
        self.news_url = news_url
        base_dir = Path.cwd()
        if str(base_dir).endswith('frontend'):
            base_dir = base_dir.parent  # move one level up in the directory tree if needed

        self.files_names = {
            self.blog_url: (base_dir / "./frontend/blog_posts.json").resolve(),
            self.news_url: (base_dir / "./frontend/news_posts.json").resolve()
        }
        # Configure session with proper cookie handling
        self.session = requests.Session()

    @staticmethod
    def get_arguments() -> Any:
        parser = argparse.ArgumentParser(description="python3 get_discourse_posts.py")
        parser.add_argument("--n_posts", dest='n_posts', default="9", help="Number of posts to retrieve")
        parser.add_argument("--verbose", dest='verbose', action='store_false', default=True, help="show debugging output")
        args, unknown = parser.parse_known_args()
        return args

    @staticmethod
    def _get_cache_path(url: str) -> Path:
        if platform.system() == "Windows":
            home = Path(os.environ.get('LOCALAPPDATA', Path.cwd()))
        else:
            home = Path(os.environ.get('HOME', Path.cwd()))
        cache_dir = home / "WebCache"
        cache_dir.mkdir(parents=True, exist_ok=True)
        url_hash = hashlib.sha256(url.encode()).hexdigest()
        return cache_dir / f"{url_hash}.json"

    def execute_http_request_json(self, url: str) -> Any:
        try:
            headers = {
                'User-Agent': 'Mozilla/5.0 (compatible; ArduPilotPostGrabber/1.0)',
                'Accept': 'application/json',
            }

            response = self.session.get(url, headers=headers, verify=True)
            response.raise_for_status()
            data = response.json()
            cache_path = BlogPostsFetcher._get_cache_path(url)
            with open(cache_path, "w", encoding="utf-8") as f:
                json.dump(data, f)
            return data
        except requests.exceptions.RequestException as err:
            cache_path = BlogPostsFetcher._get_cache_path(url)
            if cache_path.exists():
                with open(cache_path, "r", encoding="utf-8") as f:
                    return json.load(f)
            raise RequestExecutionError(f"Request failed with {err}. URL: {url}")

    @staticmethod
    def clean_html(raw_html: str) -> str:
        cleanr = re.compile('<.*?>')
        clean_text = re.sub(cleanr, '', raw_html)
        return clean_text

    @staticmethod
    def debug(str_to_print: str, verbose: bool = True) -> None:
        if verbose:
            print(f"[get_blog_posts.py] {str_to_print}")

    def get_single_post_text(self, content: Any) -> str:
        post_text = self.clean_html(content)
        item = post_text.split('\n')
        litem = " ".join(item)
        return str(litem[:140] + ' (...)')

    @staticmethod
    def get_first_youtube_or_img_link(request: str) -> Tuple[str, bool]:
        """ Returns the first YouTube link or image link in the request, if any.
            True if the link is a Youtube link."""
        request_lines = request.splitlines()
        # Join the first 5 lines back together
        first_five_lines = '\n'.join(request_lines[:5])

        # Regular expression to find URLs that contain 'YouTube' or image links
        url_pattern = re.compile(r'href=[\'"]?(https?://www\.youtube[^\'" >]+)')
        img_pattern = re.compile(r'(?i)(?:href|src)=[\'"]?(https?://[^\'" >]+\.(?:jpg|jpeg|png|gif|svg|bmp|webp))')
        img_pattern2 = re.compile(r'img src=[\'"]?(https?://[^\'" >]+)')  # catch google link and such

        # Find all matches
        youtube_links = url_pattern.findall(first_five_lines)
        img_links = img_pattern.findall(first_five_lines)[0] if img_pattern.findall(first_five_lines) else None
        if img_links is None:
            img_links = img_pattern2.findall(first_five_lines)[0] if img_pattern2.findall(
                first_five_lines) else None

        # If there are image links before YouTube links, return empty string
        if img_links and (not youtube_links or
                          first_five_lines.index(img_links) < first_five_lines.index(youtube_links[0])):
            if 'github.com' in img_links:
                img_links = img_links + "?raw=true"
            return img_links, False

        # If there are no YouTube links, still return ''
        if not youtube_links:
            return '', False

        # Return the first YouTube link, get youtube video not the playlist for thumbnail
        if '&amp;list=' in youtube_links[0]:
            return youtube_links[0].split('&amp;list=')[0], True
        return youtube_links[0], True

    @staticmethod
    def youtube_link_to_embed_link(url: str) -> str:
        return url.replace('https://www.youtube.com/watch?v=', 'https://www.youtube-nocookie.com/embed/')

    def get_post_data(self, content: Any, i: int, verbose: bool) -> Post:
        item = content['topic_list']['topics'][i]
        single_post_link = str('https://discuss.ardupilot.org/t/' + str(item['slug']) + '/' + str(item['id'])) + '.json'
        self.debug(f"Requesting post text {single_post_link} ... ", verbose)
        post_content_raw = self.execute_http_request_json(single_post_link)
        post_content = str(post_content_raw['post_stream']['posts'][0]['cooked'])
        single_post_text = self.get_single_post_text(post_content)

        has_image = False
        self.debug(f"Requesting post text {single_post_link} to look for youtube link... ", verbose)
        thing_link, isyoutube = self.get_first_youtube_or_img_link(post_content)
        youtube_link = self.youtube_link_to_embed_link(thing_link) if isyoutube else ''
        if youtube_link == '':
            if item['image_url'] is not None:
                has_image = True
                youtube_link = 'nops'
            elif thing_link != '':
                has_image = True
                youtube_link = 'nops'
                item['image_url'] = thing_link

        return Post(item['title'], item['image_url'], has_image, youtube_link, single_post_link.rsplit('.', 1)[0],
                    single_post_text.strip())

    def save_posts_to_json(self, url: str, n_posts: int, verbose: bool) -> None:
        content = self.execute_http_request_json(url)
        data = [self.get_post_data(content, i, verbose) for i in range(1, n_posts + 1)]
        self.write_to_json(url, data)

    def write_to_json(self, url: str, data: List[Post]) -> None:
        try:
            if url not in self.files_names:
                raise ValueError(f"No filename associated with url: {url}")
            post_data = [post.__dict__ for post in data]
            with open(self.files_names[url], 'w', encoding='utf-8') as f:
                json.dump(post_data, f, ensure_ascii=False, indent=4)
        except Exception as e:
            raise WriteToFileError(f"Exception occurred while writing to file with message {e}")

    def fetch(self, args: Any) -> None:
        try:
            n_posts = int(args.n_posts)
            self.debug('Starting...', args.verbose)
            self.debug(f"Requesting url {self.blog_url} ... ", args.verbose)
            self.save_posts_to_json(self.blog_url, n_posts, args.verbose)
            self.debug(f"Requesting url {self.news_url} ... ", args.verbose)
            self.save_posts_to_json(self.news_url, n_posts, args.verbose)
        except (RequestExecutionError, WriteToFileError) as e:
            print(f"Program execution failed with error: {e}")
            exit(1)


def main():
    BLOG_DISCOURSE_URL = "https://discuss.ardupilot.org/c/blog/110.json"
    NEWS_DISCOURSE_URL = "https://discuss.ardupilot.org/latest.json"
    fetcher = BlogPostsFetcher(BLOG_DISCOURSE_URL, NEWS_DISCOURSE_URL)
    args = fetcher.get_arguments()
    fetcher.fetch(args)


if __name__ == "__main__":
    main()
