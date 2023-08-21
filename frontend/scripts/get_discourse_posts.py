#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script to get last blog entries on Discourse (https://discuss.ardupilot.org/)
"""
import argparse
import json
import re
from pathlib import Path

import requests
from dataclasses import dataclass
from typing import List, Any


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
    @staticmethod
    def get_arguments() -> Any:
        parser = argparse.ArgumentParser(description="python3 get_discourse_posts.py [Number of posts to retrieve]")
        parser.add_argument("--n_posts", dest='n_posts', default="8", help="Number of posts to retrieve")
        parser.add_argument("--verbose", dest='verbose', action='store_false', default=True, help="show debugging output")
        return parser.parse_args()

    @staticmethod
    def execute_http_request_json(url: str) -> Any:
        try:
            response = requests.get(url)
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as err:
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
        post_text = self.clean_html(str(content['post_stream']['posts'][0]['cooked']))
        item = post_text.split('\n')
        litem = " ".join(item)
        return str(litem[:140] + ' (...)')

    @staticmethod
    def get_first_youtube_link(request: str) -> str:
        # regular expression to find URLs that contain 'YouTube'
        url_pattern = re.compile(r'href=[\'"]?(https?://www\.youtube[^\'" >]+)')
        youtube_links = url_pattern.findall(request)
        return youtube_links[0] if youtube_links else ''

    @staticmethod
    def youtube_link_to_embed_link(url: str) -> str:
        return str(url).replace('https://www.youtube.com/watch?v=', 'https://www.youtube-nocookie.com/embed/')

    def get_post_data(self, content: Any, i: int, verbose: bool) -> Post:
        item = content['topic_list']['topics'][i]
        single_post_link = str('https://discuss.ardupilot.org/t/' + str(item['slug']) + '/' + str(item['id'])) + '.json'
        self.debug(f"Requesting post text {single_post_link} ... ", verbose)
        post_content = self.execute_http_request_json(single_post_link)
        single_post_text = self.get_single_post_text(post_content)

        youtube_link = ''
        has_image = False
        if str(item['image_url']) == 'None':
            self.debug(f"Requesting post text {single_post_link} to look for youtube link... ", verbose)
            youtube_link = self.youtube_link_to_embed_link(self.get_first_youtube_link(single_post_text))
        else:
            has_image = True
            youtube_link = 'nops'

        return Post(item['title'], item['image_url'], has_image, youtube_link, single_post_link.rsplit('.', 1)[0], single_post_text.strip())

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
