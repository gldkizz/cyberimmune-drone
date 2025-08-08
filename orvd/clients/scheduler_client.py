import logging
import datetime
from apscheduler.schedulers.background import BackgroundScheduler
from apscheduler.job import Job as APJob
from apscheduler.jobstores.base import JobLookupError
from typing import Callable, Optional, List, Any, Dict as TypingDict
from flask import Flask

logger = logging.getLogger(__name__)

class TaskSchedulerClient:
    def __init__(self):
        self.scheduler = BackgroundScheduler()
        self.app: Optional[Flask] = None

    def init_app(self, app: Flask):
        self.app = app
        if not self.scheduler.running:
            try:
                self.scheduler.start()
                logger.info("TaskScheduler started.")
            except Exception as e:
                logger.error(f"Error starting TaskScheduler: {e}", exc_info=True)

    def _execute_with_context(self, func: Callable, args: tuple, kwargs: TypingDict[str, Any]):
        if not self.app:
            logger.error("Flask app not initialized for TaskScheduler. Task cannot run.")
            return
        
        current_app = self.app
        try:
            with current_app.app_context():
                func(*args, **kwargs)
        except Exception as e:
            logger.error(f"Error executing task {func.__name__} in app context: {e}", exc_info=True)


    def add_interval_task(
        self, 
        task_name: str, 
        seconds: int, 
        func: Callable, 
        args: Optional[tuple] = None, 
        kwargs: Optional[TypingDict[str, Any]] = None,
        job_options: Optional[TypingDict[str, Any]] = None
    ) -> Optional[APJob]:
        """
        :param task_name: A unique name/ID for the task.
        :param seconds: The interval in seconds for the task execution.
        :param func: The function to be executed.
        :param args: Positional arguments to pass to the function.
        :param kwargs: Keyword arguments to pass to the function.
        :param job_options: Additional options for APScheduler's add_job method (e.g., misfire_grace_time).
        :return: The APScheduler job object if successful, None otherwise.
        """
        if not self.app:
            logger.error("Scheduler not initialized with Flask app. Call init_app first.")
            return None

        if not isinstance(seconds, int) or seconds <= 0:
            logger.error(f"Invalid interval for task '{task_name}': seconds must be a positive integer.")
            return None

        effective_args = args if args is not None else ()
        effective_kwargs = kwargs if kwargs is not None else {}
        
        wrapped_func = lambda: self._execute_with_context(func, effective_args, effective_kwargs)

        options = job_options if job_options else {}
        options.setdefault('misfire_grace_time', 60)

        try:
            job = self.scheduler.add_job(
                wrapped_func,
                trigger='interval',
                seconds=seconds,
                id=task_name,
                name=task_name,
                replace_existing=True,
                **options
            )
            logger.info(f"Interval task '{task_name}' added/updated to run every {seconds} seconds.")
            return job
        except Exception as e:
            logger.error(f"Error adding/updating interval task '{task_name}': {e}", exc_info=True)
            return None

    def add_oneshot_task(
        self,
        task_name: str,
        seconds: int,
        func: Callable,
        args: Optional[tuple] = None,
        kwargs: Optional[TypingDict[str, Any]] = None,
        job_options: Optional[TypingDict[str, Any]] = None
    ) -> Optional[APJob]:
        if not self.app:
            logger.error("Scheduler not initialized with Flask app. Call init_app first.")
            return None

        if not isinstance(seconds, int) or seconds < 0:
            logger.error(f"Invalid delay for task '{task_name}': seconds must be a non-negative integer.")
            return None

        effective_args = args if args is not None else ()
        effective_kwargs = kwargs if kwargs is not None else {}

        wrapped_func = lambda: self._execute_with_context(func, effective_args, effective_kwargs)

        options = job_options if job_options else {}
        run_date = datetime.datetime.now() + datetime.timedelta(seconds=seconds)

        try:
            job = self.scheduler.add_job(
                wrapped_func,
                trigger='date',
                run_date=run_date,
                id=task_name,
                name=task_name,
                replace_existing=True,
                **options
            )
            logger.info(f"One-shot task '{task_name}' scheduled to run at {run_date}.")
            return job
        except Exception as e:
            logger.error(f"Error scheduling one-shot task '{task_name}': {e}", exc_info=True)
            return None

    def find_task_by_name(self, task_name: str) -> Optional[APJob]:
        try:
            job = self.scheduler.get_job(task_name)
            if job:
                logger.debug(f"Task '{task_name}' found.")
            else:
                logger.debug(f"Task '{task_name}' not found.")
            return job
        except JobLookupError:
            logger.debug(f"Task '{task_name}' not found (JobLookupError).")
            return None
        except Exception as e:
            logger.error(f"Error finding task '{task_name}': {e}", exc_info=True)
            return None

    def remove_task(self, task_name: str) -> bool:
        try:
            self.scheduler.remove_job(task_name)
            logger.info(f"Task '{task_name}' removed successfully.")
            return True
        except JobLookupError:
            logger.warning(f"Task '{task_name}' not found for removal.")
            return False
        except Exception as e:
            logger.error(f"Error removing task '{task_name}': {e}", exc_info=True)
            return False

    def get_all_tasks(self) -> List[APJob]:
        try:
            jobs = self.scheduler.get_jobs()
            logger.debug(f"Retrieved {len(jobs)} tasks.")
            return jobs
        except Exception as e:
            logger.error(f"Error retrieving all tasks: {e}", exc_info=True)
            return []

    def shutdown(self, wait: bool = True):
        try:
            if self.scheduler.running:
                self.scheduler.shutdown(wait=wait)
                logger.info(f"TaskScheduler shutdown (wait={wait}).")
        except Exception as e:
            logger.error(f"Error shutting down TaskScheduler: {e}", exc_info=True)